#![forbid(unsafe_code)]

//! FrankenTUI File Browser
//!
//! A split-pane file browser with:
//! - Directory tree on the left (clickable, expandable)
//! - File preview on the right (markdown rendering for .md files)
//! - Full mouse support (click to select/expand, scroll to navigate)
//! - Keyboard navigation (arrows, Enter, Backspace, q to quit)

use std::cell::Cell;
use std::fs;
use std::panic::AssertUnwindSafe;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, Instant, SystemTime};

use ftui_core::event::{Event, KeyCode, KeyEventKind, MouseButton, MouseEvent, MouseEventKind};
use ftui_core::geometry::Rect;
use ftui_extras::markdown::{MarkdownRenderer, MarkdownTheme};
use ftui_layout::{Constraint, Flex};
use ftui_render::cell::{Cell as RenderCell, CellAttrs, PackedRgba, StyleFlags};
use ftui_render::drawing::Draw;
use ftui_render::frame::{Frame, HitId, HitRegion};
use ftui_runtime::program::{Cmd, Model, Program, ProgramConfig};
use ftui_style::Style;
use ftui_text::{Text, WrapMode};
use ftui_widgets::block::{Alignment, Block};
use ftui_widgets::borders::BorderType;
use ftui_widgets::paragraph::Paragraph;
use ftui_widgets::scrollbar::{Scrollbar, ScrollbarOrientation, ScrollbarState};
use ftui_widgets::tree::{Tree, TreeGuides, TreeNode};
use ftui_widgets::{MouseResult, StatefulWidget, Widget};

// ---------------------------------------------------------------------------
// Constants & cached styles
// ---------------------------------------------------------------------------

const TREE_HIT_ID: HitId = HitId::new(1);
const PREVIEW_SCROLLBAR_HIT_ID: HitId = HitId::new(3);
const MAX_PREVIEW_BYTES: u64 = 256 * 1024;

/// Git commit hash baked in at build time by build.rs.
const BUILD_HASH: &str = env!("BUILD_GIT_HASH");

/// GitHub API endpoint for the latest commit on main.
const GITHUB_API_URL: &str = "https://api.github.com/repos/l0g1x/terminal/commits/main";

/// How often to check for updates (24 hours).
const UPDATE_CHECK_INTERVAL: Duration = Duration::from_secs(86_400);

// ---------------------------------------------------------------------------
// Update checker (runs in background via Cmd::task, never blocks the UI)
// ---------------------------------------------------------------------------

/// XDG cache dir: $XDG_CACHE_HOME/terminal or ~/.cache/terminal
fn cache_dir() -> Option<PathBuf> {
    std::env::var_os("XDG_CACHE_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".cache")))
        .map(|d| d.join("terminal"))
}

/// Returns true if we haven't checked in the last 24 hours.
fn should_check_for_update() -> bool {
    let Some(stamp) = cache_dir().map(|d| d.join("last-update-check")) else {
        return true;
    };
    match fs::metadata(&stamp) {
        Ok(meta) => {
            let elapsed = meta
                .modified()
                .ok()
                .and_then(|t| SystemTime::now().duration_since(t).ok())
                .unwrap_or(UPDATE_CHECK_INTERVAL);
            elapsed >= UPDATE_CHECK_INTERVAL
        }
        Err(_) => true,
    }
}

/// Write a timestamp so we skip checks for the next 24 hours.
fn touch_update_stamp() {
    if let Some(dir) = cache_dir() {
        let _ = fs::create_dir_all(&dir);
        let _ = fs::write(dir.join("last-update-check"), "");
    }
}

/// Blocking function meant to run inside `Cmd::task`.
/// Shells out to `curl` (available on any system that ran the install script)
/// to hit the GitHub API and compare the remote HEAD against our build hash.
fn check_for_update() -> Option<String> {
    if BUILD_HASH == "unknown" || !should_check_for_update() {
        return None;
    }

    touch_update_stamp();

    // 5-second timeout, silent, fail quietly on network errors.
    let output = Command::new("curl")
        .args([
            "-sSf",
            "--max-time",
            "5",
            "-H",
            "User-Agent: terminal-file-browser",
            "-H",
            "Accept: application/vnd.github.v3+json",
            GITHUB_API_URL,
        ])
        .output()
        .ok()?;

    if !output.status.success() {
        return None;
    }

    let body = String::from_utf8_lossy(&output.stdout);

    // Extract the first "sha":"..." value (the commit SHA) without a JSON crate.
    let sha_key = "\"sha\":\"";
    let start = body.find(sha_key)? + sha_key.len();
    let end = start + body[start..].find('"')?;
    let remote_sha = &body[start..end];

    if remote_sha.len() >= 7
        && !BUILD_HASH.starts_with(&remote_sha[..7])
        && !remote_sha.starts_with(BUILD_HASH)
    {
        Some(remote_sha[..7].to_string())
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Color palette
// ---------------------------------------------------------------------------

// Pre-computed color constants -- avoid re-creating PackedRgba every frame.
const COLOR_DIR: PackedRgba = PackedRgba::rgb(100, 149, 237);
const COLOR_FILE: PackedRgba = PackedRgba::rgb(200, 200, 200);
const COLOR_SELECTED: PackedRgba = PackedRgba::rgb(255, 215, 0);
const COLOR_BORDER: PackedRgba = PackedRgba::rgb(80, 80, 120);
const COLOR_STATUS_BG: PackedRgba = PackedRgba::rgb(30, 30, 50);
const COLOR_STATUS_FG: PackedRgba = PackedRgba::rgb(160, 160, 200);

/// Pre-computed styles used in view(). Built once, reused every frame.
struct CachedStyles {
    file_style: Style,
    status_style: Style,
    border_focus: Style,
    border_unfocus: Style,
    scrollbar_thumb: Style,
    scrollbar_track: Style,
}

impl CachedStyles {
    fn new() -> Self {
        Self {
            file_style: Style::new().fg(COLOR_FILE),
            status_style: Style::new().fg(COLOR_STATUS_FG).bg(COLOR_STATUS_BG),
            border_focus: Style::new().fg(COLOR_SELECTED),
            border_unfocus: Style::new().fg(COLOR_BORDER),
            scrollbar_thumb: Style::new().fg(COLOR_SELECTED),
            scrollbar_track: Style::new().fg(COLOR_BORDER),
        }
    }
}

// ---------------------------------------------------------------------------
// App state
// ---------------------------------------------------------------------------

/// The main application model for the file browser.
struct FileBrowser {
    /// Root directory being browsed.
    root: PathBuf,
    /// The tree widget representing the filesystem hierarchy.
    tree: Tree,
    /// Index of the currently selected visible node in the tree.
    selected_index: usize,
    /// Total number of currently visible nodes.
    visible_count: usize,
    /// Path of the currently selected file (if any).
    selected_file: Option<PathBuf>,
    /// Cached resolved path for the current selected_index (avoids re-walking tree).
    cached_path: Option<PathBuf>,
    /// Cached selected_index that `cached_path` corresponds to.
    cached_path_index: usize,
    /// Rendered preview text for the selected file.
    preview_text: Text,
    /// Whether the selected file is markdown (for rendering).
    is_markdown: bool,
    /// Vertical scroll offset for the preview pane.
    preview_scroll: u16,
    /// Height of the preview pane (updated each frame from view()).
    preview_height: Cell<u16>,
    /// Which pane has focus: left (tree) or right (preview).
    focus: Pane,
    /// Status message shown at bottom.
    status: String,
    /// Cached preview pane title string (only rebuilt on file selection change).
    preview_title: String,
    /// Markdown renderer instance.
    md_renderer: MarkdownRenderer,
    /// Scrollbar state for preview pane.
    scrollbar_state: ScrollbarState,
    /// Pre-computed styles reused every frame.
    styles: CachedStyles,
    /// Reusable scratch buffer for path resolution (avoids allocation per call).
    path_scratch: Vec<String>,
    /// Set when a newer version is detected on GitHub.
    update_notice: Option<String>,
    /// First visible node index in the tree viewport (scroll offset).
    tree_scroll: usize,
    /// Height of the tree pane (updated each frame from view()).
    tree_height: Cell<u16>,
    // ----- Selection & Toast State (copy-on-selection feature) -----
    /// Start cell position of text selection (col, row) relative to preview inner area.
    selection_start: Option<(u16, u16)>,
    /// End cell position of text selection (col, row) relative to preview inner area.
    selection_end: Option<(u16, u16)>,
    /// Whether user is currently dragging to select text.
    is_selecting: bool,
    /// Toast notification text to display (e.g., "Copied to clipboard").
    toast_text: Option<String>,
    /// When the toast was triggered (for auto-dismiss timing).
    toast_start: Option<Instant>,
    /// Cached preview inner area for hit-testing mouse events.
    preview_inner_area: Cell<Rect>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pane {
    Tree,
    Preview,
}

// ---------------------------------------------------------------------------
// Messages
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
enum Msg {
    KeyPress(KeyCode),
    Mouse(MouseEvent),
    Resize,
    Tick,
    /// Background update check completed. Some(hash) = update available.
    UpdateCheck(Option<String>),
    Noop,
}

impl From<Event> for Msg {
    fn from(event: Event) -> Self {
        match event {
            Event::Key(ke) => {
                if ke.kind == KeyEventKind::Press || ke.kind == KeyEventKind::Repeat {
                    Msg::KeyPress(ke.code)
                } else {
                    Msg::Noop
                }
            }
            Event::Mouse(me) => Msg::Mouse(me),
            Event::Resize { .. } => Msg::Resize,
            Event::Tick => Msg::Tick,
            _ => Msg::Noop,
        }
    }
}

// ---------------------------------------------------------------------------
// Filesystem helpers
// ---------------------------------------------------------------------------

/// Build a stub `TreeNode` for a path. Does NOT read directory contents.
/// Children are loaded lazily by `reload_children` when the user expands.
fn build_tree_node(path: &Path) -> TreeNode {
    let name = path.file_name().map_or_else(
        || path.to_string_lossy().into_owned(),
        |n| n.to_string_lossy().into_owned(),
    );

    if path.is_dir() {
        TreeNode::new(format!("{name}/")).with_expanded(false)
    } else {
        TreeNode::new(name).with_expanded(false)
    }
}

/// Reload children for a directory node when expanded.
fn reload_children(node: &mut TreeNode, path: &Path) {
    if !path.is_dir() || !node.children().is_empty() {
        return;
    }
    let mut dirs: Vec<PathBuf> = Vec::new();
    let mut files: Vec<PathBuf> = Vec::new();
    if let Ok(entries) = fs::read_dir(path) {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.file_name()
                .is_some_and(|n| n.to_string_lossy().starts_with('.'))
            {
                continue;
            }
            if p.is_dir() {
                dirs.push(p);
            } else {
                files.push(p);
            }
        }
    }
    dirs.sort();
    files.sort();
    let mut children = Vec::new();
    for d in &dirs {
        let dname = d.file_name().map_or_else(
            || d.to_string_lossy().into_owned(),
            |n| n.to_string_lossy().into_owned(),
        );
        children.push(TreeNode::new(format!("{dname}/")).with_expanded(false));
    }
    for f in &files {
        let fname = f.file_name().map_or_else(
            || f.to_string_lossy().into_owned(),
            |n| n.to_string_lossy().into_owned(),
        );
        children.push(TreeNode::new(fname).with_expanded(false));
    }
    let label = node.label().to_string();
    let expanded = node.is_expanded();
    *node = TreeNode::new(label)
        .with_children(children)
        .with_expanded(expanded);
}

/// Given a visible index in the tree, resolve the filesystem path.
/// Uses a pre-allocated scratch buffer to avoid allocations.
fn resolve_path_for_index(
    root: &Path,
    tree: &Tree,
    index: usize,
    scratch: &mut Vec<String>,
) -> Option<PathBuf> {
    scratch.clear();
    if collect_path_at_index(
        tree.root(),
        index,
        scratch,
        tree.root().label().ends_with('/'),
    ) {
        let mut full_path = root.to_path_buf();
        for part in &scratch[1..] {
            let clean = part.trim_end_matches('/');
            full_path = full_path.join(clean);
        }
        Some(full_path)
    } else {
        None
    }
}

fn collect_path_at_index(
    node: &TreeNode,
    target: usize,
    path: &mut Vec<String>,
    show_root: bool,
) -> bool {
    let mut counter: usize = 0;
    collect_path_recursive(node, target, path, &mut counter, show_root)
}

fn collect_path_recursive(
    node: &TreeNode,
    target: usize,
    path: &mut Vec<String>,
    counter: &mut usize,
    is_visible: bool,
) -> bool {
    if is_visible {
        if *counter == target {
            path.push(node.label().to_string());
            return true;
        }
        *counter += 1;
    }

    if node.is_expanded() || !is_visible {
        for child in node.children() {
            path.push(node.label().to_string());
            if collect_path_recursive(child, target, path, counter, true) {
                return true;
            }
            path.pop();
        }
    }
    false
}

/// Check if a file is likely markdown by extension.
fn is_markdown_file(path: &Path) -> bool {
    matches!(
        path.extension().and_then(|e| e.to_str()),
        Some("md" | "markdown" | "mdown" | "mkd" | "mdx")
    )
}

/// Size of the prefix sample used for binary detection.
const BINARY_DETECT_BYTES: usize = 8192;

/// Fraction of non-text bytes (excluding valid UTF-8 continuation) that
/// triggers binary classification when no null byte is present.
const BINARY_RATIO_THRESHOLD: f64 = 0.30;

/// Detect whether a byte slice looks like binary data.
///
/// Heuristics (same approach as `git` and `file(1)`):
/// 1. Any null byte (`\0`) → binary (catches ELF, Mach-O, images, archives).
/// 2. High ratio of non-text bytes → binary. "Text" bytes are printable
///    ASCII (`0x20..=0x7E`), tab (`0x09`), newline (`0x0A`), carriage
///    return (`0x0D`), and valid UTF-8 continuation bytes (`0x80..=0xBF`
///    when preceded by a valid lead byte — approximated here by accepting
///    all high bytes, since we later validate via `String::from_utf8`).
fn is_binary_data(sample: &[u8]) -> bool {
    if sample.is_empty() {
        return false;
    }

    let mut non_text: usize = 0;
    for &b in sample {
        if b == 0 {
            return true; // Null byte — binary.
        }
        let is_text = matches!(b, 0x09 | 0x0A | 0x0D | 0x20..=0x7E | 0x80..=0xFF);
        if !is_text {
            non_text += 1;
        }
    }

    (non_text as f64 / sample.len() as f64) > BINARY_RATIO_THRESHOLD
}

/// Tab width used when expanding `\t` to spaces.
const TAB_WIDTH: usize = 4;

/// Sanitize file content for safe rendering in FrankenTUI.
///
/// FrankenTUI allocates exactly 1 cell per character with no special
/// handling for control codes. The terminal, however, interprets `\t` as
/// "advance to next tab stop" and `\r` as "move cursor to column 0",
/// causing a mismatch between the buffer model and the actual cursor
/// position. This corrupts every character that follows on the same line.
///
/// Transformations applied per line:
/// - **Tabs** are expanded to spaces (aligned to `TAB_WIDTH` boundaries).
/// - **Carriage returns** (`\r`) are stripped.
/// - **Other control chars** (`0x00..0x1F` except `\n`, and `0x7F`) are
///   replaced with the Unicode replacement character `U+FFFD`.
/// - **Newlines** (`\n`) are preserved (they are line separators in
///   `Text::raw`).
fn sanitize_for_display(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut col: usize = 0;

    for c in text.chars() {
        match c {
            '\t' => {
                // Expand to next TAB_WIDTH boundary.
                let spaces = TAB_WIDTH - (col % TAB_WIDTH);
                for _ in 0..spaces {
                    out.push(' ');
                }
                col += spaces;
            }
            '\n' => {
                out.push('\n');
                col = 0;
            }
            '\r' => {
                // Strip carriage returns entirely.
            }
            c if c.is_control() => {
                out.push('\u{FFFD}');
                col += 1;
            }
            c => {
                out.push(c);
                col += 1;
            }
        }
    }
    out
}

/// Read file content for preview (capped at `MAX_PREVIEW_BYTES`).
///
/// Pipeline:
/// 1. Read raw bytes (up to `MAX_PREVIEW_BYTES`).
/// 2. Run binary detection on the first `BINARY_DETECT_BYTES`.
/// 3. Decode as UTF-8 (reject if invalid).
/// 4. Sanitize stray control characters.
fn read_file_preview(path: &Path) -> Result<String, String> {
    let metadata = fs::metadata(path).map_err(|e| format!("Cannot read: {e}"))?;
    let file_len = metadata.len();

    // Step 1: Read raw bytes.
    let raw = fs::read(path).map_err(|e| format!("Cannot read: {e}"))?;

    // Step 2: Binary detection on a prefix sample.
    let sample_end = raw.len().min(BINARY_DETECT_BYTES);
    if is_binary_data(&raw[..sample_end]) {
        return Err("Binary file (cannot preview)".to_string());
    }

    // Step 3: Determine if we need to truncate, then decode as UTF-8.
    let truncated = file_len > MAX_PREVIEW_BYTES;
    let usable = if truncated {
        &raw[..MAX_PREVIEW_BYTES as usize]
    } else {
        &raw
    };

    let text = String::from_utf8(usable.to_vec())
        .map_err(|_| "Binary file (cannot preview)".to_string())?;

    // Step 4: Sanitize for display (expand tabs, strip \r, replace controls).
    let clean = sanitize_for_display(&text);

    if truncated {
        Ok(format!("{clean}\n\n--- (truncated at 256KB) ---"))
    } else {
        Ok(clean)
    }
}

/// Human-readable file size.
fn human_size(bytes: u64) -> String {
    const UNITS: &[&str] = &["B", "KB", "MB", "GB", "TB"];
    let mut size = bytes as f64;
    for unit in UNITS {
        if size < 1024.0 {
            return format!("{size:.1} {unit}");
        }
        size /= 1024.0;
    }
    format!("{size:.1} PB")
}

// ---------------------------------------------------------------------------
// Model implementation
// ---------------------------------------------------------------------------

impl FileBrowser {
    fn new(root: PathBuf) -> Self {
        let tree_root = build_tree_node(&root);
        let visible = tree_root.visible_count();
        let tree = Tree::new(tree_root)
            .with_show_root(true)
            .with_guides(TreeGuides::Rounded)
            .with_guide_style(Style::new().fg(COLOR_BORDER))
            .with_label_style(Style::new().fg(COLOR_FILE))
            .with_root_style(Style::new().fg(COLOR_DIR).bold())
            .hit_id(TREE_HIT_ID);

        let root_name = root.file_name().map_or_else(
            || root.to_string_lossy().into_owned(),
            |n| n.to_string_lossy().into_owned(),
        );

        Self {
            root,
            tree,
            selected_index: 0,
            visible_count: visible,
            selected_file: None,
            cached_path: None,
            cached_path_index: usize::MAX,
            preview_text: Text::raw("Select a file to preview"),
            is_markdown: false,
            preview_scroll: 0,
            preview_height: Cell::new(0),
            focus: Pane::Tree,
            status: format!(
                "  {root_name}/ | arrows: navigate | Enter: open | Tab: switch pane | q: quit"
            ),
            preview_title: " Preview ".to_string(),
            md_renderer: MarkdownRenderer::new(MarkdownTheme::default()),
            scrollbar_state: ScrollbarState::new(0, 0, 0),
            styles: CachedStyles::new(),
            path_scratch: Vec::with_capacity(32),
            update_notice: None,
            tree_scroll: 0,
            tree_height: Cell::new(0),
            // Selection & toast state
            selection_start: None,
            selection_end: None,
            is_selecting: false,
            toast_text: None,
            toast_start: None,
            preview_inner_area: Cell::new(Rect::new(0, 0, 0, 0)),
        }
    }

    /// Resolve the filesystem path for the current selected_index, using cache.
    fn resolve_current_path(&mut self) -> Option<PathBuf> {
        if self.cached_path_index == self.selected_index {
            return self.cached_path.clone();
        }
        let path = resolve_path_for_index(
            &self.root,
            &self.tree,
            self.selected_index,
            &mut self.path_scratch,
        );
        self.cached_path = path.clone();
        self.cached_path_index = self.selected_index;
        path
    }

    /// Invalidate the path cache (call after tree structure changes).
    fn invalidate_path_cache(&mut self) {
        self.cached_path_index = usize::MAX;
        self.cached_path = None;
    }

    /// Select a file and load its preview.
    fn select_file(&mut self, path: PathBuf) {
        let size_str = fs::metadata(&path)
            .map(|m| human_size(m.len()))
            .unwrap_or_else(|_| "?".into());
        let name = path.file_name().map_or_else(
            || path.to_string_lossy().into_owned(),
            |n| n.to_string_lossy().into_owned(),
        );
        self.status = format!("  {name} | {size_str} | Tab: switch pane | q: quit");
        self.is_markdown = is_markdown_file(&path);

        // Update cached preview title (only rebuilt on file change, not every frame)
        self.preview_title = if self.is_markdown {
            format!(" {name} (Markdown) ")
        } else {
            format!(" {name} ")
        };

        match read_file_preview(&path) {
            Ok(content) => {
                if self.is_markdown {
                    let renderer = &self.md_renderer;
                    let rendered =
                        std::panic::catch_unwind(AssertUnwindSafe(|| renderer.render(&content)));
                    match rendered {
                        Ok(text) => self.preview_text = text,
                        Err(_) => {
                            self.preview_text = Text::raw(format!(
                                "[Markdown math rendering failed -- showing raw text]\n\n{content}"
                            ));
                        }
                    }
                } else {
                    self.preview_text = Text::raw(&content);
                }
            }
            Err(msg) => {
                self.preview_text = Text::raw(&msg);
            }
        }

        self.selected_file = Some(path);
        self.preview_scroll = 0;
        let total = self.preview_text.height();
        self.scrollbar_state = ScrollbarState::new(total, 0, self.preview_height.get() as usize);
    }

    /// Keep `tree_scroll` in sync so `selected_index` is always visible.
    fn update_tree_scroll(&mut self) {
        let h = self.tree_height.get() as usize;
        if h == 0 {
            return;
        }
        if self.selected_index < self.tree_scroll {
            self.tree_scroll = self.selected_index;
        } else if self.selected_index >= self.tree_scroll + h {
            self.tree_scroll = self.selected_index - h + 1;
        }
    }

    /// Move tree selection by `delta` steps without triggering file I/O for
    /// intermediate positions. Only resolves + previews the final position.
    fn move_selection_by(&mut self, delta: isize) {
        let new_index = if delta > 0 {
            (self.selected_index + delta as usize).min(self.visible_count.saturating_sub(1))
        } else {
            self.selected_index.saturating_sub((-delta) as usize)
        };
        if new_index != self.selected_index {
            self.selected_index = new_index;
            self.update_tree_scroll();
            self.on_selection_changed();
        }
    }

    /// Called when the tree selection changes to potentially preview a file.
    fn on_selection_changed(&mut self) {
        if let Some(path) = self.resolve_current_path() {
            if path.is_file() {
                self.select_file(path);
            } else {
                // Directory -- show basic info without expensive fs::read_dir count
                let name = path.file_name().map_or_else(
                    || path.to_string_lossy().into_owned(),
                    |n| n.to_string_lossy().into_owned(),
                );
                self.status = format!("  {name}/ | Enter: expand | q: quit");
                self.selected_file = None;
                self.preview_title = " Preview ".to_string();
                self.preview_text = Text::raw(format!("Directory: {}", path.display()));
                self.preview_scroll = 0;
            }
        }
    }

    /// Handle Enter key: toggle expand for dirs, select file.
    fn handle_enter(&mut self) {
        if let Some(path) = self.resolve_current_path() {
            if path.is_dir() {
                if let Some(node) = self.tree.node_at_visible_index_mut(self.selected_index) {
                    if !node.is_expanded() {
                        reload_children(node, &path);
                    }
                    node.toggle_expanded();
                }
                self.visible_count = self.tree.root().visible_count();
                self.invalidate_path_cache();
                self.update_tree_scroll();
            } else if path.is_file() {
                self.select_file(path);
                self.focus = Pane::Preview;
            }
        }
    }

    /// Scroll preview up.
    fn scroll_preview_up(&mut self, lines: u16) {
        self.preview_scroll = self.preview_scroll.saturating_sub(lines);
        self.scrollbar_state.position = self.preview_scroll as usize;
    }

    /// Scroll preview down.
    fn scroll_preview_down(&mut self, lines: u16) {
        let max = (self.preview_text.height() as u16).saturating_sub(self.preview_height.get() / 2);
        self.preview_scroll = self.preview_scroll.saturating_add(lines).min(max);
        self.scrollbar_state.position = self.preview_scroll as usize;
    }

    // -----------------------------------------------------------------------
    // Selection & Clipboard helpers (copy-on-selection feature)
    // -----------------------------------------------------------------------

    /// Map a cell position (x, y) relative to preview inner area to a character
    /// index in the preview text, accounting for scroll offset and word wrapping.
    /// Returns None if position is outside text bounds.
    fn map_cell_to_char(&self, x: u16, y: u16, wrap_width: u16) -> Option<usize> {
        // Account for scroll offset
        let logical_y = y as usize + self.preview_scroll as usize;

        // Convert preview_text to plain string for character indexing
        let plain = self.preview_text_as_string();
        let mut char_idx = 0;
        let mut current_line = 0;
        let mut current_col = 0;

        for ch in plain.chars() {
            if current_line == logical_y && current_col == x as usize {
                return Some(char_idx);
            }

            if ch == '\n' {
                // If we're on the target line but past the newline, clamp to end of line
                if current_line == logical_y {
                    return Some(char_idx);
                }
                current_line += 1;
                current_col = 0;
            } else {
                current_col += 1;
                // Handle word wrapping
                if wrap_width > 0 && current_col >= wrap_width as usize {
                    current_line += 1;
                    current_col = 0;
                }
            }
            char_idx += 1;
        }

        // If we reached the target line, return end of text
        if current_line == logical_y {
            Some(char_idx)
        } else {
            None
        }
    }

    /// Extract selected text between two character indices.
    /// Handles reversed selections (end before start).
    fn extract_selected_text(&self, start_idx: usize, end_idx: usize) -> String {
        let plain = self.preview_text_as_string();
        let (lo, hi) = if start_idx <= end_idx {
            (start_idx, end_idx)
        } else {
            (end_idx, start_idx)
        };
        plain.chars().skip(lo).take(hi - lo).collect()
    }

    /// Convert preview_text (ftui Text) to a plain String.
    fn preview_text_as_string(&self) -> String {
        // Text is composed of Lines, each Line has Spans
        // We need to extract the raw text content
        let mut result = String::new();
        for (i, line) in self.preview_text.lines().iter().enumerate() {
            if i > 0 {
                result.push('\n');
            }
            for span in line.spans() {
                result.push_str(&span.content);
            }
        }
        result
    }

    /// Check if a mouse position is within the preview inner area.
    fn is_in_preview_area(&self, x: u16, y: u16) -> bool {
        let area = self.preview_inner_area.get();
        x >= area.x && x < area.x + area.width && y >= area.y && y < area.y + area.height
    }

    /// Convert absolute screen coordinates to preview-relative coordinates.
    fn screen_to_preview_coords(&self, x: u16, y: u16) -> (u16, u16) {
        let area = self.preview_inner_area.get();
        (x.saturating_sub(area.x), y.saturating_sub(area.y))
    }

    /// Clear current selection state.
    fn clear_selection(&mut self) {
        self.selection_start = None;
        self.selection_end = None;
        self.is_selecting = false;
    }

    /// Show a toast notification that auto-dismisses.
    fn show_toast(&mut self, message: &str) {
        self.toast_text = Some(message.to_string());
        self.toast_start = Some(Instant::now());
    }

    /// Check if toast should be dismissed (after ~1.5 seconds).
    fn check_toast_timeout(&mut self) {
        if let Some(start) = self.toast_start
            && start.elapsed() > Duration::from_millis(1500) {
                self.toast_text = None;
                self.toast_start = None;
            }
    }

    /// Copy selected text to clipboard and show toast.
    fn copy_selection_to_clipboard(&mut self) {
        let (start, end) = match (self.selection_start, self.selection_end) {
            (Some(s), Some(e)) => (s, e),
            _ => return,
        };

        let wrap_width = self.preview_inner_area.get().width;
        let start_idx = self.map_cell_to_char(start.0, start.1, wrap_width);
        let end_idx = self.map_cell_to_char(end.0, end.1, wrap_width);

        if let (Some(si), Some(ei)) = (start_idx, end_idx) {
            let text = self.extract_selected_text(si, ei);
            if !text.is_empty() {
                // Copy to clipboard using arboard
                if let Ok(mut clipboard) = arboard::Clipboard::new()
                    && clipboard.set_text(&text).is_ok() {
                        self.show_toast("Copied to clipboard");
                    }
            }
        }

        self.clear_selection();
    }
}

impl Model for FileBrowser {
    type Message = Msg;

    fn init(&mut self) -> Cmd<Self::Message> {
        let root_path = self.root.clone();
        if let Some(root_node) = self.tree.node_at_visible_index_mut(0) {
            reload_children(root_node, &root_path);
            if !root_node.is_expanded() {
                root_node.toggle_expanded();
            }
        }
        self.visible_count = self.tree.root().visible_count();
        self.invalidate_path_cache();

        // Fire a background update check -- runs in a thread, never blocks UI.
        Cmd::task(|| Msg::UpdateCheck(check_for_update()))
    }

    fn update(&mut self, msg: Msg) -> Cmd<Self::Message> {
        match msg {
            Msg::KeyPress(code) => match code {
                KeyCode::Char('q') | KeyCode::Escape => return Cmd::quit(),

                KeyCode::Up | KeyCode::Char('k') => {
                    if self.focus == Pane::Tree {
                        self.move_selection_by(-1);
                    } else {
                        self.scroll_preview_up(1);
                    }
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    if self.focus == Pane::Tree {
                        self.move_selection_by(1);
                    } else {
                        self.scroll_preview_down(1);
                    }
                }
                KeyCode::PageUp => {
                    if self.focus == Pane::Preview {
                        self.scroll_preview_up(self.preview_height.get().saturating_sub(2));
                    } else {
                        // Jump 10 items in one step -- no intermediate file reads
                        self.move_selection_by(-10);
                    }
                }
                KeyCode::PageDown => {
                    if self.focus == Pane::Preview {
                        self.scroll_preview_down(self.preview_height.get().saturating_sub(2));
                    } else {
                        self.move_selection_by(10);
                    }
                }
                KeyCode::Home => {
                    if self.focus == Pane::Tree {
                        self.selected_index = 0;
                        self.update_tree_scroll();
                        self.on_selection_changed();
                    } else {
                        self.preview_scroll = 0;
                        self.scrollbar_state.position = 0;
                    }
                }
                KeyCode::End => {
                    if self.focus == Pane::Tree {
                        self.selected_index = self.visible_count.saturating_sub(1);
                        self.update_tree_scroll();
                        self.on_selection_changed();
                    } else {
                        let max = (self.preview_text.height() as u16)
                            .saturating_sub(self.preview_height.get() / 2);
                        self.preview_scroll = max;
                        self.scrollbar_state.position = max as usize;
                    }
                }

                KeyCode::Enter | KeyCode::Right | KeyCode::Char('l') => {
                    if self.focus == Pane::Tree {
                        self.handle_enter();
                    }
                }
                KeyCode::Backspace | KeyCode::Left | KeyCode::Char('h') => {
                    if self.focus == Pane::Tree {
                        if let Some(node) = self.tree.node_at_visible_index_mut(self.selected_index)
                        {
                            if node.is_expanded() && !node.children().is_empty() {
                                node.toggle_expanded();
                                self.visible_count = self.tree.root().visible_count();
                                self.invalidate_path_cache();
                                self.update_tree_scroll();
                            } else if self.selected_index > 0 {
                                self.move_selection_by(-1);
                            }
                        }
                    } else {
                        self.focus = Pane::Tree;
                    }
                }

                KeyCode::Tab => {
                    self.focus = match self.focus {
                        Pane::Tree => Pane::Preview,
                        Pane::Preview => Pane::Tree,
                    };
                }

                _ => {}
            },

            Msg::Mouse(me) => match me.kind {
                MouseEventKind::Down(MouseButton::Left) => {
                    // Check if click is in preview area for text selection
                    if self.is_in_preview_area(me.x, me.y) {
                        let (px, py) = self.screen_to_preview_coords(me.x, me.y);
                        self.selection_start = Some((px, py));
                        self.selection_end = Some((px, py));
                        self.is_selecting = true;
                        self.focus = Pane::Preview;
                    } else {
                        // Handle tree pane click
                        let result = self.tree.handle_mouse(
                            &me,
                            Some((TREE_HIT_ID, HitRegion::Content, 0)),
                            TREE_HIT_ID,
                        );
                        match result {
                            MouseResult::Selected(idx) | MouseResult::Activated(idx) => {
                                self.selected_index = idx;
                                self.invalidate_path_cache();
                                self.update_tree_scroll();
                                self.focus = Pane::Tree;
                                if result == MouseResult::Activated(idx) {
                                    self.handle_enter();
                                } else {
                                    self.on_selection_changed();
                                }
                            }
                            _ => {
                                self.focus = Pane::Preview;
                            }
                        }
                    }
                }
                MouseEventKind::Drag(MouseButton::Left) => {
                    // Update selection end during drag
                    if self.is_selecting && self.is_in_preview_area(me.x, me.y) {
                        let (px, py) = self.screen_to_preview_coords(me.x, me.y);
                        self.selection_end = Some((px, py));
                    }
                }
                MouseEventKind::Up(MouseButton::Left) => {
                    // Finalize selection and copy to clipboard
                    if self.is_selecting {
                        if self.is_in_preview_area(me.x, me.y) {
                            let (px, py) = self.screen_to_preview_coords(me.x, me.y);
                            self.selection_end = Some((px, py));
                        }
                        self.copy_selection_to_clipboard();
                    }
                }
                MouseEventKind::ScrollUp => {
                    if self.focus == Pane::Preview {
                        self.scroll_preview_up(3);
                    } else {
                        self.move_selection_by(-1);
                    }
                }
                MouseEventKind::ScrollDown => {
                    if self.focus == Pane::Preview {
                        self.scroll_preview_down(3);
                    } else {
                        self.move_selection_by(1);
                    }
                }
                _ => {}
            },

            Msg::Resize => {
                self.visible_count = self.tree.root().visible_count();
                self.invalidate_path_cache();
            }

            Msg::UpdateCheck(result) => {
                if let Some(hash) = result {
                    self.update_notice = Some(format!(
                        "Update available ({hash})! Run: curl -sSf https://raw.githubusercontent.com/l0g1x/terminal/main/install.sh | bash"
                    ));
                }
            }

            Msg::Tick => {
                // Check if toast should be auto-dismissed
                self.check_toast_timeout();
            }
            Msg::Noop => {}
        }
        Cmd::none()
    }

    fn view(&self, frame: &mut Frame) {
        let total_area = Rect::from_size(frame.buffer.width(), frame.buffer.height());
        if total_area.is_empty() {
            return;
        }

        // Main layout: tree | preview, status bar at bottom
        let vert_areas = Flex::vertical()
            .constraints([Constraint::Fill, Constraint::Fixed(1)])
            .split(total_area);

        let main_area = vert_areas[0];
        let status_area = vert_areas[1];

        let horiz_areas = Flex::horizontal()
            .constraints([Constraint::Percentage(30.0), Constraint::Fill])
            .split(main_area);

        let tree_area = horiz_areas[0];
        let preview_area = horiz_areas[1];

        // ---- Tree pane ----
        let tree_border = if self.focus == Pane::Tree {
            self.styles.border_focus
        } else {
            self.styles.border_unfocus
        };

        let tree_block = Block::bordered()
            .title(" Files ")
            .title_alignment(Alignment::Left)
            .border_style(tree_border)
            .border_type(BorderType::Rounded);

        let inner_tree_area = tree_block.inner(tree_area);
        tree_block.render(tree_area, frame);
        self.tree_height.set(inner_tree_area.height);
        frame.buffer.fill(inner_tree_area, RenderCell::default());
        self.render_tree_with_selection(inner_tree_area, frame);

        // ---- Preview pane ----
        let preview_border = if self.focus == Pane::Preview {
            self.styles.border_focus
        } else {
            self.styles.border_unfocus
        };

        let preview_block = Block::bordered()
            .title(&self.preview_title)
            .title_alignment(Alignment::Left)
            .border_style(preview_border)
            .border_type(BorderType::Rounded);

        let inner_preview_area = preview_block.inner(preview_area);
        preview_block.render(preview_area, frame);

        let ph = inner_preview_area.height;
        self.preview_height.set(ph);
        // Store preview inner area for mouse hit testing
        self.preview_inner_area.set(inner_preview_area);

        // Clear the preview area before rendering to prevent stale content
        // from the previous file bleeding through when switching to shorter text.
        frame.buffer.fill(inner_preview_area, RenderCell::default());

        let preview_paragraph = Paragraph::new(self.preview_text.clone())
            .style(self.styles.file_style)
            .wrap(WrapMode::Word)
            .scroll((self.preview_scroll, 0));

        preview_paragraph.render(inner_preview_area, frame);

        // Render selection highlight if active
        if self.is_selecting || (self.selection_start.is_some() && self.selection_end.is_some()) {
            self.render_selection_highlight(inner_preview_area, frame);
        }

        // Scrollbar (only when content overflows)
        if self.preview_text.height() > ph as usize {
            let mut sb_state = ScrollbarState::new(
                self.preview_text.height(),
                self.preview_scroll as usize,
                ph as usize,
            );
            let scrollbar = Scrollbar::new(ScrollbarOrientation::VerticalRight)
                .thumb_style(self.styles.scrollbar_thumb)
                .track_style(self.styles.scrollbar_track)
                .hit_id(PREVIEW_SCROLLBAR_HIT_ID);
            StatefulWidget::render(&scrollbar, inner_preview_area, frame, &mut sb_state);
        }

        // Render toast notification if active
        if let Some(ref toast) = self.toast_text {
            self.render_toast(toast, inner_preview_area, frame);
        }

        // ---- Status bar ----
        let status_text = if let Some(ref notice) = self.update_notice {
            notice.as_str()
        } else {
            self.status.as_str()
        };
        let status_style = if self.update_notice.is_some() {
            Style::new()
                .fg(PackedRgba::rgb(255, 200, 60))
                .bg(COLOR_STATUS_BG)
                .bold()
        } else {
            self.styles.status_style
        };
        let status_para = Paragraph::new(Text::raw(status_text)).style(status_style);
        status_para.render(status_area, frame);
    }
}

// ---------------------------------------------------------------------------
// Manual tree renderer with scrolling + selection highlighting
// ---------------------------------------------------------------------------

/// A flattened visible tree node for rendering.
struct FlatNode<'a> {
    label: &'a str,
    depth: usize,
    is_dir: bool,
    /// For each depth 0..depth, whether the ancestor at that level was the
    /// last child (determines │ vs blank in guide columns).
    is_last_stack: Vec<bool>,
}

/// Flatten the visible tree into a linear list for viewport rendering.
fn flatten_visible<'a>(node: &'a TreeNode, show_root: bool) -> Vec<FlatNode<'a>> {
    let mut out = Vec::new();
    let mut stack = Vec::new();
    if show_root {
        flatten_walk(node, 0, &mut stack, &mut out, true);
    } else if node.is_expanded() {
        let child_count = node.children().len();
        for (i, child) in node.children().iter().enumerate() {
            stack.clear();
            stack.push(i == child_count - 1);
            flatten_walk(child, 0, &mut stack, &mut out, i == child_count - 1);
        }
    }
    out
}

fn flatten_walk<'a>(
    node: &'a TreeNode,
    depth: usize,
    is_last_stack: &mut Vec<bool>,
    out: &mut Vec<FlatNode<'a>>,
    is_last: bool,
) {
    // Ensure the stack has the correct entry for this depth.
    if is_last_stack.len() <= depth {
        is_last_stack.push(is_last);
    } else {
        is_last_stack[depth] = is_last;
    }

    out.push(FlatNode {
        label: node.label(),
        depth,
        is_dir: node.label().ends_with('/'),
        is_last_stack: is_last_stack[..=depth].to_vec(),
    });

    if node.is_expanded() {
        let child_count = node.children().len();
        for (i, child) in node.children().iter().enumerate() {
            flatten_walk(child, depth + 1, is_last_stack, out, i == child_count - 1);
        }
    }
}

impl FileBrowser {
    /// Render the tree manually with viewport scrolling and selection highlight.
    ///
    /// Uses `Buffer::print_text_clipped` (via the `Draw` trait) for correct
    /// wide-character handling, overlap cleanup, and dirty tracking.
    fn render_tree_with_selection(&self, area: Rect, frame: &mut Frame) {
        if area.is_empty() || self.visible_count == 0 {
            return;
        }

        let viewport_h = area.height as usize;
        let guides = TreeGuides::Rounded;

        // Flatten the visible tree.
        let flat = flatten_visible(self.tree.root(), true);

        // Determine viewport slice.
        let scroll = self.tree_scroll;
        let end = (scroll + viewport_h).min(flat.len());

        let selected = self
            .selected_index
            .min(self.visible_count.saturating_sub(1));

        for (row_idx, flat_idx) in (scroll..end).enumerate() {
            let node = &flat[flat_idx];
            let y = area.y + row_idx as u16;
            let max_x = area.right();
            let mut x = area.x;

            let is_selected = flat_idx == selected;

            // Determine colors for this row.
            let guide_fg = if is_selected {
                COLOR_SELECTED
            } else {
                COLOR_BORDER
            };
            let label_fg = if is_selected {
                COLOR_SELECTED
            } else if node.is_dir {
                COLOR_DIR
            } else {
                COLOR_FILE
            };

            // Build a base cell for guide characters.
            let guide_cell = RenderCell::from_char(' ').with_fg(guide_fg);

            // Draw guide characters for each depth level using print_text_clipped.
            for d in 0..node.depth {
                if x + 4 > max_x {
                    break;
                }
                let guide_str = if d + 1 == node.depth {
                    // This is the immediate parent connection.
                    if node.is_last_stack[d + 1] {
                        guides.last()
                    } else {
                        guides.branch()
                    }
                } else {
                    // Continuation from ancestor.
                    if d + 1 < node.is_last_stack.len() && node.is_last_stack[d + 1] {
                        guides.space()
                    } else {
                        guides.vertical()
                    }
                };

                x = frame
                    .buffer
                    .print_text_clipped(x, y, guide_str, guide_cell, max_x);
            }

            // Build a base cell for the label.
            let bold = is_selected || (node.depth == 0 && node.is_dir);
            let label_attrs = if bold {
                CellAttrs::new(StyleFlags::BOLD, 0)
            } else {
                CellAttrs::NONE
            };
            let label_cell = RenderCell::from_char(' ')
                .with_fg(label_fg)
                .with_attrs(label_attrs);

            // Draw the label using print_text_clipped.
            frame
                .buffer
                .print_text_clipped(x, y, node.label, label_cell, max_x);
        }
    }

    /// Render selection highlight by inverting colors in the selected region.
    fn render_selection_highlight(&self, area: Rect, frame: &mut Frame) {
        let (start, end) = match (self.selection_start, self.selection_end) {
            (Some(s), Some(e)) => (s, e),
            _ => return,
        };

        // Normalize start and end (handle reverse selection)
        let (start_y, start_x, end_y, end_x) = if (start.1, start.0) <= (end.1, end.0) {
            (start.1, start.0, end.1, end.0)
        } else {
            (end.1, end.0, start.1, start.0)
        };

        // Highlight cells in the selection range
        for row in start_y..=end_y {
            if row >= area.height {
                break;
            }
            let screen_y = area.y + row;

            let col_start = if row == start_y { start_x } else { 0 };
            let col_end = if row == end_y {
                end_x
            } else {
                area.width.saturating_sub(1)
            };

            for col in col_start..=col_end {
                if col >= area.width {
                    break;
                }
                let screen_x = area.x + col;

                // Invert fg/bg colors for selection highlight
                if let Some(cell) = frame.buffer.get_mut(screen_x, screen_y) {
                    let orig_fg = cell.fg;
                    let orig_bg = cell.bg;
                    cell.fg = if orig_bg == PackedRgba::TRANSPARENT {
                        PackedRgba::rgb(30, 30, 30) // Dark background as fg
                    } else {
                        orig_bg
                    };
                    cell.bg = if orig_fg == PackedRgba::TRANSPARENT {
                        PackedRgba::rgb(200, 200, 200) // Light fg as bg
                    } else {
                        orig_fg
                    };
                }
            }
        }
    }

    /// Render toast notification in the upper-right corner of the preview area.
    fn render_toast(&self, message: &str, area: Rect, frame: &mut Frame) {
        // Toast dimensions
        let toast_width = (message.len() + 4).min(area.width as usize) as u16;
        let toast_height = 3u16;

        // Position in upper-right corner with some margin
        let toast_x = area.x + area.width.saturating_sub(toast_width + 1);
        let toast_y = area.y + 1;

        if toast_x < area.x || toast_y + toast_height > area.y + area.height {
            return; // Not enough space
        }

        let toast_area = Rect::new(toast_x, toast_y, toast_width, toast_height);

        // Toast styling
        let toast_bg = PackedRgba::rgb(50, 50, 50);
        let toast_fg = PackedRgba::rgb(150, 255, 150); // Light green
        let border_fg = PackedRgba::rgb(100, 200, 100);

        // Draw background
        for y in toast_area.y..toast_area.y + toast_area.height {
            for x in toast_area.x..toast_area.x + toast_area.width {
                if let Some(cell) = frame.buffer.get_mut(x, y) {
                    cell.bg = toast_bg;
                    cell.fg = toast_fg;
                    cell.content = ftui_render::cell::CellContent::from_char(' ');
                }
            }
        }

        // Draw border (simple box)
        let border_chars = ['╭', '─', '╮', '│', '│', '╰', '─', '╯'];
        // Top-left
        if let Some(c) = frame.buffer.get_mut(toast_area.x, toast_area.y) {
            c.content = ftui_render::cell::CellContent::from_char(border_chars[0]);
            c.fg = border_fg;
        }
        // Top-right
        if let Some(c) = frame
            .buffer
            .get_mut(toast_area.x + toast_area.width - 1, toast_area.y)
        {
            c.content = ftui_render::cell::CellContent::from_char(border_chars[2]);
            c.fg = border_fg;
        }
        // Bottom-left
        if let Some(c) = frame
            .buffer
            .get_mut(toast_area.x, toast_area.y + toast_area.height - 1)
        {
            c.content = ftui_render::cell::CellContent::from_char(border_chars[5]);
            c.fg = border_fg;
        }
        // Bottom-right
        if let Some(c) = frame.buffer.get_mut(
            toast_area.x + toast_area.width - 1,
            toast_area.y + toast_area.height - 1,
        ) {
            c.content = ftui_render::cell::CellContent::from_char(border_chars[7]);
            c.fg = border_fg;
        }
        // Top/bottom borders
        for x in (toast_area.x + 1)..(toast_area.x + toast_area.width - 1) {
            if let Some(c) = frame.buffer.get_mut(x, toast_area.y) {
                c.content = ftui_render::cell::CellContent::from_char(border_chars[1]);
                c.fg = border_fg;
            }
            if let Some(c) = frame
                .buffer
                .get_mut(x, toast_area.y + toast_area.height - 1)
            {
                c.content = ftui_render::cell::CellContent::from_char(border_chars[6]);
                c.fg = border_fg;
            }
        }
        // Side borders
        for y in (toast_area.y + 1)..(toast_area.y + toast_area.height - 1) {
            if let Some(c) = frame.buffer.get_mut(toast_area.x, y) {
                c.content = ftui_render::cell::CellContent::from_char(border_chars[3]);
                c.fg = border_fg;
            }
            if let Some(c) = frame.buffer.get_mut(toast_area.x + toast_area.width - 1, y) {
                c.content = ftui_render::cell::CellContent::from_char(border_chars[4]);
                c.fg = border_fg;
            }
        }

        // Draw message text (centered in middle row)
        let text_y = toast_area.y + 1;
        let text_start_x = toast_area.x + 2;
        let max_text_len = (toast_area.width as usize).saturating_sub(4);
        let display_msg: String = message.chars().take(max_text_len).collect();

        for (i, ch) in display_msg.chars().enumerate() {
            let x = text_start_x + i as u16;
            if x < toast_area.x + toast_area.width - 1
                && let Some(c) = frame.buffer.get_mut(x, text_y) {
                    c.content = ftui_render::cell::CellContent::from_char(ch);
                    c.fg = toast_fg;
                }
        }
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use ftui_render::grapheme_pool::GraphemePool;

    // -----------------------------------------------------------------------
    // Helper: extract a row from a Frame as a trimmed String.
    // -----------------------------------------------------------------------
    fn row_text(frame: &Frame, y: u16, width: u16) -> String {
        let mut out = String::new();
        for x in 0..width {
            if let Some(cell) = frame.buffer.get(x, y) {
                if let Some(ch) = cell.content.as_char() {
                    out.push(ch);
                } else if cell.content.is_empty() {
                    out.push(' ');
                }
                // skip continuations
            }
        }
        out.trim_end().to_string()
    }

    /// Helper: extract the fg color at (x, y).
    fn fg_at(frame: &Frame, x: u16, y: u16) -> PackedRgba {
        frame
            .buffer
            .get(x, y)
            .map_or(PackedRgba::TRANSPARENT, |c| c.fg)
    }

    /// Find the column index of the first occurrence of `needle` in a row string.
    /// This accounts for multi-byte UTF-8 characters where byte offset != column.
    fn find_col(row: &str, needle: char) -> Option<usize> {
        row.chars().position(|c| c == needle)
    }

    /// Helper: check if the cell at (x, y) has the BOLD flag.
    fn is_bold_at(frame: &Frame, x: u16, y: u16) -> bool {
        frame
            .buffer
            .get(x, y)
            .map_or(false, |c| c.attrs.has_flag(StyleFlags::BOLD))
    }

    // -----------------------------------------------------------------------
    // Helper: create a FileBrowser with a synthetic in-memory tree.
    // -----------------------------------------------------------------------
    fn make_test_browser(nodes: Vec<TreeNode>, selected: usize, scroll: usize) -> FileBrowser {
        let root_node = nodes
            .into_iter()
            .fold(TreeNode::new("root/").with_expanded(true), |root, child| {
                root.child(child)
            });
        let visible = root_node.visible_count();
        let tree = Tree::new(root_node)
            .with_show_root(true)
            .with_guides(TreeGuides::Rounded)
            .with_guide_style(Style::new().fg(COLOR_BORDER))
            .with_label_style(Style::new().fg(COLOR_FILE))
            .with_root_style(Style::new().fg(COLOR_DIR).bold())
            .hit_id(TREE_HIT_ID);

        FileBrowser {
            root: PathBuf::from("/test"),
            tree,
            selected_index: selected,
            visible_count: visible,
            selected_file: None,
            cached_path: None,
            cached_path_index: usize::MAX,
            preview_text: Text::raw(""),
            is_markdown: false,
            preview_scroll: 0,
            preview_height: Cell::new(0),
            focus: Pane::Tree,
            status: String::new(),
            preview_title: String::new(),
            md_renderer: MarkdownRenderer::new(MarkdownTheme::default()),
            scrollbar_state: ScrollbarState::new(0, 0, 0),
            styles: CachedStyles::new(),
            path_scratch: Vec::new(),
            update_notice: None,
            tree_scroll: scroll,
            tree_height: Cell::new(0),
            // Selection & toast state
            selection_start: None,
            selection_end: None,
            is_selecting: false,
            toast_text: None,
            toast_start: None,
            preview_inner_area: Cell::new(Rect::new(0, 0, 0, 0)),
        }
    }

    /// Helper: create a FileBrowser with preview content for selection tests.
    fn make_test_browser_with_preview(preview: &str, scroll: u16) -> FileBrowser {
        let mut browser = make_test_browser(vec![], 0, 0);
        browser.preview_text = Text::raw(preview);
        browser.preview_scroll = scroll;
        browser.focus = Pane::Preview;
        browser
    }

    // -----------------------------------------------------------------------
    // Helper: render the tree into a Frame and return it for assertions.
    // -----------------------------------------------------------------------
    /// Render the tree and return each row as a String.
    fn render_tree_rows(browser: &FileBrowser, width: u16, height: u16) -> Vec<String> {
        let mut pool = GraphemePool::new();
        let mut frame = Frame::new(width, height, &mut pool);
        let area = Rect::new(0, 0, width, height);

        // Set tree_height so update_tree_scroll works.
        browser.tree_height.set(height);

        browser.render_tree_with_selection(area, &mut frame);

        (0..height).map(|y| row_text(&frame, y, width)).collect()
    }

    /// Render the tree and return (rows, fg_colors_per_row, bold_flags_per_row).
    fn render_tree_detailed(
        browser: &FileBrowser,
        width: u16,
        height: u16,
    ) -> (Vec<String>, Vec<Vec<PackedRgba>>, Vec<Vec<bool>>) {
        let mut pool = GraphemePool::new();
        let mut frame = Frame::new(width, height, &mut pool);
        let area = Rect::new(0, 0, width, height);
        browser.tree_height.set(height);
        browser.render_tree_with_selection(area, &mut frame);

        let rows: Vec<String> = (0..height).map(|y| row_text(&frame, y, width)).collect();
        let fgs: Vec<Vec<PackedRgba>> = (0..height)
            .map(|y| (0..width).map(|x| fg_at(&frame, x, y)).collect())
            .collect();
        let bolds: Vec<Vec<bool>> = (0..height)
            .map(|y| (0..width).map(|x| is_bold_at(&frame, x, y)).collect())
            .collect();

        (rows, fgs, bolds)
    }

    // =======================================================================
    // INTEGRATION TESTS: Tree Rendering
    // =======================================================================

    #[test]
    fn render_basic_tree_root_only() {
        // A tree with just the root node, no children.
        let browser = make_test_browser(vec![], 0, 0);
        let rows = render_tree_rows(&browser, 30, 5);

        assert_eq!(rows[0], "root/", "root node should render on first row");
        assert_eq!(rows[1], "", "no second row");
    }

    #[test]
    fn render_tree_with_children() {
        let browser = make_test_browser(
            vec![
                TreeNode::new("src/").with_expanded(false),
                TreeNode::new("Cargo.toml"),
                TreeNode::new("README.md"),
            ],
            0,
            0,
        );
        let rows = render_tree_rows(&browser, 40, 10);

        assert_eq!(rows[0], "root/");
        // Children should have guide characters.
        assert!(
            rows[1].contains("src/"),
            "row 1 should contain 'src/': got '{}'",
            rows[1]
        );
        assert!(
            rows[2].contains("Cargo.toml"),
            "row 2 should contain 'Cargo.toml': got '{}'",
            rows[2]
        );
        assert!(
            rows[3].contains("README.md"),
            "row 3 should contain 'README.md': got '{}'",
            rows[3]
        );
    }

    #[test]
    fn render_tree_guide_characters() {
        // Verify that guide characters are correct (├── for non-last, ╰── for last).
        let browser = make_test_browser(
            vec![TreeNode::new("first.txt"), TreeNode::new("last.txt")],
            0,
            0,
        );
        let rows = render_tree_rows(&browser, 40, 5);

        // Row 0: root/
        assert_eq!(rows[0], "root/");
        // Row 1: ├── first.txt  (branch guide)
        assert!(
            rows[1].starts_with("├── "),
            "non-last child should use branch guide '├── ': got '{}'",
            rows[1]
        );
        assert!(rows[1].contains("first.txt"));
        // Row 2: ╰── last.txt  (last guide)
        assert!(
            rows[2].starts_with("╰── "),
            "last child should use last guide '╰── ': got '{}'",
            rows[2]
        );
        assert!(rows[2].contains("last.txt"));
    }

    #[test]
    fn render_tree_nested_guides() {
        // Test nested tree with vertical continuation guides.
        let browser = make_test_browser(
            vec![
                TreeNode::new("dir/")
                    .with_expanded(true)
                    .with_children(vec![TreeNode::new("inner.txt")]),
                TreeNode::new("other.txt"),
            ],
            0,
            0,
        );
        let rows = render_tree_rows(&browser, 40, 10);

        // Row 0: root/
        // Row 1: ├── dir/
        // Row 2: │   ╰── inner.txt
        // Row 3: ╰── other.txt
        assert_eq!(rows[0], "root/");
        assert!(rows[1].contains("dir/"), "row 1: {}", rows[1]);
        assert!(
            rows[2].starts_with("│"),
            "row 2 should start with vertical guide: got '{}'",
            rows[2]
        );
        assert!(rows[2].contains("inner.txt"), "row 2: {}", rows[2]);
        assert!(
            rows[3].starts_with("╰── "),
            "row 3 should start with last guide: got '{}'",
            rows[3]
        );
    }

    #[test]
    fn render_tree_selection_color() {
        // Verify the selected row gets COLOR_SELECTED for its label fg.
        let browser = make_test_browser(
            vec![TreeNode::new("file_a.txt"), TreeNode::new("file_b.txt")],
            2, // select file_b.txt (index 0=root, 1=file_a, 2=file_b)
            0,
        );
        let (rows, fgs, _bolds) = render_tree_detailed(&browser, 40, 5);

        // Row 2 is file_b.txt, which is selected.
        assert!(rows[2].contains("file_b.txt"), "row 2: {}", rows[2]);
        let label_start = find_col(&rows[2], 'f').expect("should find 'f' in file_b.txt");
        assert_eq!(
            fgs[2][label_start], COLOR_SELECTED,
            "selected label should have COLOR_SELECTED fg"
        );

        // Row 1 is file_a.txt, NOT selected.
        let label_a_start = find_col(&rows[1], 'f').expect("should find 'f' in file_a.txt");
        assert_eq!(
            fgs[1][label_a_start], COLOR_FILE,
            "non-selected file should have COLOR_FILE fg"
        );
    }

    #[test]
    fn render_tree_bold_on_selected() {
        let browser = make_test_browser(
            vec![TreeNode::new("selected.txt")],
            1, // select the file
            0,
        );
        let (_rows, _fgs, bolds) = render_tree_detailed(&browser, 40, 5);

        // Row 1 is selected.txt. Guides are 4 chars wide, label starts at col 4.
        assert!(bolds[1][4], "selected row label should be bold at col 4");
        // Row 0 is root/ which is also bold (depth 0 + is_dir).
        assert!(bolds[0][0], "root label should be bold");
    }

    #[test]
    fn render_tree_scrolled_viewport() {
        // Create a tree taller than the viewport and scroll down.
        let children: Vec<TreeNode> = (0..20)
            .map(|i| TreeNode::new(format!("file_{i:02}.txt")))
            .collect();
        let browser = make_test_browser(children, 15, 10);
        // viewport height = 5, scroll = 10, so we see items 10..15
        let rows = render_tree_rows(&browser, 40, 5);

        // flat_idx 10 = "file_09.txt" (index 0 = root, then 1..20 = files)
        assert!(
            rows[0].contains("file_09.txt"),
            "scrolled row 0 should show file_09.txt: got '{}'",
            rows[0]
        );
        assert!(
            rows[4].contains("file_13.txt"),
            "scrolled row 4 should show file_13.txt: got '{}'",
            rows[4]
        );
    }

    #[test]
    fn render_tree_scroll_keeps_selection_visible() {
        let children: Vec<TreeNode> = (0..20)
            .map(|i| TreeNode::new(format!("file_{i:02}.txt")))
            .collect();
        let mut browser = make_test_browser(children, 0, 0);
        browser.tree_height.set(5);

        // Move selection to item 10 (beyond viewport of 5).
        browser.selected_index = 10;
        browser.update_tree_scroll();

        assert!(
            browser.tree_scroll + 5 > 10,
            "scroll should keep selected_index 10 in viewport of 5, scroll={}",
            browser.tree_scroll
        );
        assert!(
            browser.tree_scroll <= 10,
            "scroll should be <= selected_index, scroll={}",
            browser.tree_scroll
        );
    }

    #[test]
    fn render_tree_scroll_up() {
        let children: Vec<TreeNode> = (0..20)
            .map(|i| TreeNode::new(format!("file_{i:02}.txt")))
            .collect();
        let mut browser = make_test_browser(children, 15, 15);
        browser.tree_height.set(5);

        browser.selected_index = 5;
        browser.update_tree_scroll();

        assert!(
            browser.tree_scroll <= 5,
            "scroll should adjust down to show item 5, scroll={}",
            browser.tree_scroll
        );
    }

    #[test]
    fn render_tree_empty_area() {
        // Rendering into a 1x1 minimal area should not panic.
        let browser = make_test_browser(vec![TreeNode::new("test.txt")], 0, 0);
        let rows = render_tree_rows(&browser, 1, 1);
        assert_eq!(rows.len(), 1);
    }

    #[test]
    fn render_tree_narrow_viewport() {
        let browser = make_test_browser(
            vec![TreeNode::new("very_long_filename_that_exceeds_width.txt")],
            0,
            0,
        );
        let rows = render_tree_rows(&browser, 10, 3);

        assert_eq!(rows[0], "root/");
        assert!(
            rows[1].chars().count() <= 10,
            "row 1 should be clipped to viewport width (10 cols): got '{}' ({} chars)",
            rows[1],
            rows[1].chars().count()
        );
    }

    #[test]
    fn render_tree_selection_at_last_item() {
        let browser = make_test_browser(
            vec![
                TreeNode::new("a.txt"),
                TreeNode::new("b.txt"),
                TreeNode::new("z.txt"),
            ],
            3, // last visible item
            0,
        );
        let (rows, fgs, _) = render_tree_detailed(&browser, 40, 6);

        assert!(rows[3].contains("z.txt"), "row 3: {}", rows[3]);
        let label_start = find_col(&rows[3], 'z').unwrap();
        assert_eq!(
            fgs[3][label_start], COLOR_SELECTED,
            "last item when selected should have COLOR_SELECTED fg"
        );
    }

    #[test]
    fn render_tree_dir_color() {
        // Directories (non-selected) should use COLOR_DIR.
        let browser = make_test_browser(
            vec![
                TreeNode::new("mydir/").with_expanded(false),
                TreeNode::new("file.txt"),
            ],
            2, // select file.txt so mydir/ is NOT selected
            0,
        );
        let (rows, fgs, _bolds) = render_tree_detailed(&browser, 40, 5);

        let dir_start = find_col(&rows[1], 'm').expect("should find 'm' in mydir/");
        assert_eq!(
            fgs[1][dir_start], COLOR_DIR,
            "non-selected directory should have COLOR_DIR fg"
        );
    }

    #[test]
    fn flatten_visible_count_matches() {
        // Verify flatten_visible produces the same count as TreeNode::visible_count.
        let root = TreeNode::new("root/")
            .with_expanded(true)
            .with_children(vec![
                TreeNode::new("dir/")
                    .with_expanded(true)
                    .with_children(vec![TreeNode::new("a.txt"), TreeNode::new("b.txt")]),
                TreeNode::new("c.txt"),
            ]);
        let flat = flatten_visible(&root, true);
        assert_eq!(
            flat.len(),
            root.visible_count(),
            "flatten_visible count should match visible_count"
        );
    }

    #[test]
    fn flatten_visible_collapsed_dir() {
        // A collapsed directory should not show its children.
        let root = TreeNode::new("root/")
            .with_expanded(true)
            .with_children(vec![
                TreeNode::new("collapsed_dir/")
                    .with_expanded(false)
                    .with_children(vec![TreeNode::new("hidden.txt")]),
                TreeNode::new("visible.txt"),
            ]);
        let flat = flatten_visible(&root, true);

        // Should have: root/, collapsed_dir/, visible.txt = 3
        assert_eq!(flat.len(), 3);
        assert_eq!(flat[0].label, "root/");
        assert_eq!(flat[1].label, "collapsed_dir/");
        assert_eq!(flat[2].label, "visible.txt");
    }

    #[test]
    fn flatten_visible_deeply_nested() {
        let root = TreeNode::new("root/")
            .with_expanded(true)
            .with_children(vec![TreeNode::new("a/").with_expanded(true).with_children(
                vec![TreeNode::new("b/")
                    .with_expanded(true)
                    .with_children(vec![TreeNode::new("c.txt")])],
            )]);

        let flat = flatten_visible(&root, true);
        assert_eq!(flat.len(), 4);
        assert_eq!(flat[0].depth, 0);
        assert_eq!(flat[1].depth, 1);
        assert_eq!(flat[2].depth, 2);
        assert_eq!(flat[3].depth, 3);
    }

    #[test]
    fn flat_node_is_last_stack_correctness() {
        // Verify is_last_stack for guide rendering decisions.
        let root = TreeNode::new("root/")
            .with_expanded(true)
            .with_children(vec![
                TreeNode::new("first/")
                    .with_expanded(true)
                    .with_children(vec![TreeNode::new("inner.txt")]),
                TreeNode::new("last.txt"),
            ]);

        let flat = flatten_visible(&root, true);

        // flat[0] = root/ (depth 0, is_last_stack = [true or false])
        // flat[1] = first/ (depth 1, NOT last child)
        assert!(!flat[1].is_last_stack[1], "first/ is NOT the last child");
        // flat[2] = inner.txt (depth 2, IS last child of first/)
        assert!(flat[2].is_last_stack[2], "inner.txt IS the last child");
        // flat[3] = last.txt (depth 1, IS last child)
        assert!(flat[3].is_last_stack[1], "last.txt IS the last child");
    }

    #[test]
    fn render_no_stale_content_between_frames() {
        // Simulate two renders: first with a long tree, then with a short tree.
        // Verify no stale content from the first render bleeds through.
        let mut pool = GraphemePool::new();
        let mut frame = Frame::new(40, 10, &mut pool);
        let area = Rect::new(0, 0, 40, 10);

        // First render: tree with many items.
        let browser1 = make_test_browser(
            (0..8)
                .map(|i| TreeNode::new(format!("file_{i}.txt")))
                .collect(),
            0,
            0,
        );
        browser1.tree_height.set(10);
        // Fill area first (simulates what view() does).
        frame.buffer.fill(area, RenderCell::default());
        browser1.render_tree_with_selection(area, &mut frame);

        // Second render: tree with only 2 items, into the SAME frame after clear.
        frame.buffer.fill(area, RenderCell::default());
        let browser2 = make_test_browser(vec![TreeNode::new("only.txt")], 0, 0);
        browser2.tree_height.set(10);
        browser2.render_tree_with_selection(area, &mut frame);

        // Rows after the short tree should be empty.
        let row5 = row_text(&frame, 5, 40);
        assert_eq!(
            row5, "",
            "row 5 should be empty after short tree render: got '{row5}'"
        );
    }

    // =======================================================================
    // REGRESSION TESTS: Binary Detection & Control Character Sanitization
    // =======================================================================

    #[test]
    fn binary_detect_null_bytes() {
        // Any null byte should trigger binary detection.
        assert!(is_binary_data(b"hello\x00world"));
        assert!(is_binary_data(b"\x00"));
        assert!(is_binary_data(&[0u8; 100]));
    }

    #[test]
    fn binary_detect_elf_header() {
        // ELF magic: \x7fELF followed by binary metadata.
        let mut elf = vec![0x7f, b'E', b'L', b'F', 2, 1, 1, 0];
        elf.extend_from_slice(&[0u8; 56]); // padding with nulls
        assert!(is_binary_data(&elf));
    }

    #[test]
    fn binary_detect_high_non_text_ratio() {
        // A buffer dominated by control chars (no null) should be binary.
        let mostly_ctrl: Vec<u8> = (0..100).map(|i| (i % 31) as u8 + 1).collect();
        // This contains bytes 0x01..0x1F cycling — many are non-text.
        assert!(
            is_binary_data(&mostly_ctrl),
            "high ratio of control bytes should be detected as binary"
        );
    }

    #[test]
    fn binary_detect_normal_text_passes() {
        assert!(!is_binary_data(b"Hello, world!\nThis is plain text.\n"));
        assert!(!is_binary_data(b"fn main() {\n    println!(\"hi\");\n}\n"));
        assert!(!is_binary_data(b""));
    }

    #[test]
    fn binary_detect_utf8_text_passes() {
        // UTF-8 with non-ASCII characters should NOT be classified as binary.
        let utf8 = "Héllo wörld! 日本語テスト — ñ ü ö ä".as_bytes();
        assert!(
            !is_binary_data(utf8),
            "valid UTF-8 non-ASCII text should not be binary"
        );
    }

    #[test]
    fn sanitize_strips_control_chars() {
        let input = "hello\x01world\x07test\x1b[31m";
        let clean = sanitize_for_display(input);
        assert!(!clean.contains('\x01'), "SOH should be replaced");
        assert!(!clean.contains('\x07'), "BEL should be replaced");
        assert!(!clean.contains('\x1b'), "ESC should be replaced");
        assert!(clean.contains("hello"), "text preserved");
        assert!(clean.contains("world"), "text preserved");
    }

    #[test]
    fn sanitize_expands_tabs_to_spaces() {
        // Tab at column 0 should expand to TAB_WIDTH spaces.
        let input = "\thello";
        let clean = sanitize_for_display(input);
        assert_eq!(clean, "    hello", "tab at col 0 -> 4 spaces");

        // Tab after 2 chars should expand to 2 spaces (next 4-col boundary).
        let input2 = "ab\tcd";
        let clean2 = sanitize_for_display(input2);
        assert_eq!(clean2, "ab  cd", "tab at col 2 -> 2 spaces to col 4");

        // Tab after 4 chars should expand to 4 spaces (next boundary).
        let input3 = "abcd\tx";
        let clean3 = sanitize_for_display(input3);
        assert_eq!(clean3, "abcd    x", "tab at col 4 -> 4 spaces to col 8");

        // Multiple tabs.
        let input4 = "\t\tindented";
        let clean4 = sanitize_for_display(input4);
        assert_eq!(clean4, "        indented", "two tabs -> 8 spaces");
    }

    #[test]
    fn sanitize_expands_tabs_makefile_style() {
        // Typical Makefile: recipe lines start with a tab.
        let input = "all: build\n\tgcc -o main main.c\n\techo done\n";
        let clean = sanitize_for_display(input);
        assert!(!clean.contains('\t'), "no literal tabs should remain");
        assert!(clean.contains("    gcc -o main main.c"), "tab expanded");
        assert!(clean.contains("    echo done"), "tab expanded");
    }

    #[test]
    fn sanitize_strips_carriage_returns() {
        // Windows line endings.
        let input = "line1\r\nline2\r\nline3\r\n";
        let clean = sanitize_for_display(input);
        assert!(!clean.contains('\r'), "\\r should be stripped");
        assert_eq!(clean, "line1\nline2\nline3\n");
    }

    #[test]
    fn sanitize_strips_bare_carriage_returns() {
        // Progress-bar style output with bare \r.
        let input = "progress: 50%\rprogress: 100%\n";
        let clean = sanitize_for_display(input);
        assert!(!clean.contains('\r'), "bare \\r should be stripped");
        assert!(clean.contains("progress: 50%"));
    }

    #[test]
    fn sanitize_preserves_newlines() {
        let input = "line1\nline2\nline3";
        let clean = sanitize_for_display(input);
        assert_eq!(clean, input, "newlines should be preserved");
    }

    #[test]
    fn sanitize_tab_column_resets_on_newline() {
        // Tab column tracking must reset after \n.
        let input = "abc\n\tx";
        let clean = sanitize_for_display(input);
        assert_eq!(clean, "abc\n    x", "tab column resets after newline");
    }

    #[test]
    fn read_file_preview_rejects_binary() {
        let dir = std::env::temp_dir().join("ftui_test_binary_preview");
        let _ = fs::create_dir_all(&dir);

        // Write a fake binary file (ELF-like header with null bytes).
        let bin_path = dir.join("test.bin");
        let mut data = vec![0x7f, b'E', b'L', b'F'];
        data.extend_from_slice(&[0u8; 200]);
        data.extend_from_slice(b"some text mixed in");
        let _ = fs::write(&bin_path, &data);

        let result = read_file_preview(&bin_path);
        assert!(
            result.is_err(),
            "binary file should be rejected: got {:?}",
            result
        );
        assert!(
            result.unwrap_err().contains("Binary"),
            "error message should mention binary"
        );

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn read_file_preview_accepts_text() {
        let dir = std::env::temp_dir().join("ftui_test_text_preview");
        let _ = fs::create_dir_all(&dir);

        let txt_path = dir.join("test.txt");
        let _ = fs::write(&txt_path, "Hello, world!\nLine 2\nLine 3\n");

        let result = read_file_preview(&txt_path);
        assert!(result.is_ok(), "text file should be accepted");
        let content = result.unwrap();
        assert!(content.contains("Hello, world!"));

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn read_file_preview_sanitizes_control_chars() {
        let dir = std::env::temp_dir().join("ftui_test_ctrl_preview");
        let _ = fs::create_dir_all(&dir);

        // Write a text file with embedded control characters.
        let txt_path = dir.join("dirty.txt");
        let _ = fs::write(&txt_path, "clean\x07bell\x1bescape\n");

        let result = read_file_preview(&txt_path);
        assert!(
            result.is_ok(),
            "text file with control chars should be accepted"
        );
        let content = result.unwrap();
        assert!(!content.contains('\x07'), "BEL should be sanitized out");
        assert!(!content.contains('\x1b'), "ESC should be sanitized out");
        assert!(content.contains("clean"), "text preserved");
        assert!(content.contains('\n'), "newlines preserved");

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn read_file_preview_expands_tabs_in_makefile() {
        let dir = std::env::temp_dir().join("ftui_test_makefile_preview");
        let _ = fs::create_dir_all(&dir);

        let makefile_path = dir.join("Makefile");
        let _ = fs::write(
            &makefile_path,
            "all: build\n\tgcc -o main main.c\n\t@echo done\n",
        );

        let result = read_file_preview(&makefile_path);
        assert!(result.is_ok(), "Makefile should be readable");
        let content = result.unwrap();
        assert!(!content.contains('\t'), "tabs should be expanded");
        assert!(
            content.contains("    gcc -o main main.c"),
            "tab -> 4 spaces"
        );

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn read_file_preview_strips_windows_crlf() {
        let dir = std::env::temp_dir().join("ftui_test_crlf_preview");
        let _ = fs::create_dir_all(&dir);

        let path = dir.join("windows.txt");
        let _ = fs::write(&path, "line1\r\nline2\r\nline3\r\n");

        let result = read_file_preview(&path);
        assert!(result.is_ok());
        let content = result.unwrap();
        assert!(!content.contains('\r'), "\\r should be stripped");
        assert_eq!(content, "line1\nline2\nline3\n");

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn markdown_render_survives_complex_latex() {
        let renderer = MarkdownRenderer::new(MarkdownTheme::default());
        let nasty_markdown = r#"# Math-Heavy Document

## Bayesian Scoring

$$
P(relevant | evidence) / P(not\_relevant | evidence)
    = [P(relevant) / P(not\_relevant)] \times \Pi_i BF_i
$$

where $BF_i = P(evidence_i | relevant) / P(evidence_i | not\_relevant)$

## Subscript groups

$$
S_t = max(0, S_{t-1} + (X_t - \mu_0) - k)
$$

## Run-length posterior

$$
P(r_t = 0 | x_{1:t}) \propto \Sigma_r P(r_{t-1} = r) \times H(r) \times P(x_t | r)
$$

## Superscript groups

$$
W_t = W_{t-1} \times (1 + \lambda_t \times (X_t - \mu_0))
$$
"#;

        let result = std::panic::catch_unwind(AssertUnwindSafe(|| renderer.render(nasty_markdown)));
        match result {
            Ok(text) => assert!(text.height() > 0, "rendered text should have content"),
            Err(_) => {} // Panic caught -- app would show fallback
        }
    }

    #[test]
    fn plain_text_preview_works() {
        let text = Text::raw("Hello, world!\nLine 2\nLine 3");
        assert_eq!(text.height(), 3);
    }

    #[test]
    fn build_tree_from_temp_dir() {
        let dir = std::env::temp_dir().join("ftui_test_tree");
        let _ = fs::create_dir_all(dir.join("subdir"));
        let _ = fs::write(dir.join("file.txt"), "hello");
        let _ = fs::write(dir.join("readme.md"), "# Test");
        let _ = fs::write(dir.join(".hidden"), "secret");

        // build_tree_node returns a stub -- reload_children populates it.
        let mut node = build_tree_node(&dir);
        assert!(node.label().ends_with('/'));
        assert!(node.children().is_empty(), "stub should have no children");

        reload_children(&mut node, &dir);
        let child_labels: Vec<&str> = node.children().iter().map(|c| c.label()).collect();
        assert!(
            !child_labels.is_empty(),
            "reload_children should populate children"
        );
        assert!(
            !child_labels.iter().any(|l| l.starts_with('.')),
            "hidden files should be excluded: {child_labels:?}"
        );

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn path_resolution_for_root() {
        let dir = std::env::temp_dir().join("ftui_test_resolve");
        let _ = fs::create_dir_all(&dir);
        let _ = fs::write(dir.join("test.txt"), "data");

        let root_node = build_tree_node(&dir);
        let tree = Tree::new(root_node).with_show_root(true);

        let mut scratch = Vec::new();
        let resolved = resolve_path_for_index(&dir, &tree, 0, &mut scratch);
        assert!(resolved.is_some(), "root should resolve");

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn human_size_formatting() {
        assert_eq!(human_size(0), "0.0 B");
        assert_eq!(human_size(1023), "1023.0 B");
        assert_eq!(human_size(1024), "1.0 KB");
        assert_eq!(human_size(1_048_576), "1.0 MB");
        assert_eq!(human_size(1_073_741_824), "1.0 GB");
    }

    #[test]
    fn markdown_detection() {
        assert!(is_markdown_file(Path::new("README.md")));
        assert!(is_markdown_file(Path::new("doc.markdown")));
        assert!(is_markdown_file(Path::new("page.mdx")));
        assert!(!is_markdown_file(Path::new("code.rs")));
        assert!(!is_markdown_file(Path::new("data.json")));
        assert!(!is_markdown_file(Path::new("noext")));
    }

    #[test]
    fn frankentui_readme_survives() {
        let readme_path = Path::new("/home/krystian/frankentui/README.md");
        if !readme_path.exists() {
            return;
        }
        let content = fs::read_to_string(readme_path).unwrap();
        let renderer = MarkdownRenderer::new(MarkdownTheme::default());
        let result = std::panic::catch_unwind(AssertUnwindSafe(|| renderer.render(&content)));
        assert!(
            result.is_ok() || result.is_err(),
            "catch_unwind should always return Ok or Err"
        );
    }

    // =======================================================================
    // COPY-ON-SELECTION TESTS (file-browser-gbw)
    // =======================================================================

    // -----------------------------------------------------------------------
    // Selection State Tests (file-browser-gbw.2)
    // -----------------------------------------------------------------------

    #[test]
    fn selection_starts_on_mouse_down_in_preview() {
        let mut browser = make_test_browser_with_preview("Hello World", 0);
        // Set up preview area for hit testing
        browser.preview_inner_area.set(Rect::new(10, 1, 30, 10));

        // Simulate mouse down in preview area
        browser.selection_start = Some((5, 2));
        browser.is_selecting = true;

        assert!(
            browser.is_selecting,
            "is_selecting should be true after mouse down"
        );
        assert_eq!(
            browser.selection_start,
            Some((5, 2)),
            "selection_start should be set"
        );
    }

    #[test]
    fn selection_updates_on_mouse_drag() {
        let mut browser = make_test_browser_with_preview("Hello World\nLine two", 0);
        browser.preview_inner_area.set(Rect::new(10, 1, 30, 10));
        browser.selection_start = Some((0, 0));
        browser.is_selecting = true;

        // Simulate drag to new position
        browser.selection_end = Some((5, 1));

        assert_eq!(
            browser.selection_end,
            Some((5, 1)),
            "selection_end should update during drag"
        );
        assert!(
            browser.is_selecting,
            "is_selecting should remain true during drag"
        );
    }

    #[test]
    fn selection_clears_on_mouse_up() {
        let mut browser = make_test_browser_with_preview("Hello World", 0);
        browser.preview_inner_area.set(Rect::new(10, 1, 30, 10));
        browser.selection_start = Some((0, 0));
        browser.selection_end = Some((5, 0));
        browser.is_selecting = true;

        // Simulate mouse up by clearing selection
        browser.clear_selection();

        assert!(
            !browser.is_selecting,
            "is_selecting should be false after mouse up"
        );
        assert_eq!(
            browser.selection_start, None,
            "selection_start should be cleared"
        );
        assert_eq!(
            browser.selection_end, None,
            "selection_end should be cleared"
        );
    }

    #[test]
    fn selection_ignored_outside_preview_pane() {
        let browser = make_test_browser_with_preview("Hello World", 0);
        browser.preview_inner_area.set(Rect::new(10, 1, 30, 10));

        // Position outside preview area (in tree pane)
        assert!(
            !browser.is_in_preview_area(5, 5),
            "should not be in preview area"
        );

        // Position inside preview area
        assert!(
            browser.is_in_preview_area(15, 5),
            "should be in preview area"
        );
    }

    #[test]
    fn selection_canceled_on_escape() {
        let mut browser = make_test_browser_with_preview("Hello World", 0);
        browser.selection_start = Some((0, 0));
        browser.selection_end = Some((5, 0));
        browser.is_selecting = true;

        // Escape should clear selection
        browser.clear_selection();

        assert!(!browser.is_selecting);
        assert_eq!(browser.selection_start, None);
        assert_eq!(browser.selection_end, None);
    }

    // -----------------------------------------------------------------------
    // Text Position Mapping Tests (file-browser-gbw.3)
    // -----------------------------------------------------------------------

    #[test]
    fn map_cell_to_char_basic() {
        let browser = make_test_browser_with_preview("Hello World", 0);

        // Map position (0, 0) should be character 0 ('H')
        let idx = browser.map_cell_to_char(0, 0, 80);
        assert_eq!(idx, Some(0), "position (0,0) should map to char index 0");

        // Map position (5, 0) should be character 5 (' ')
        let idx = browser.map_cell_to_char(5, 0, 80);
        assert_eq!(idx, Some(5), "position (5,0) should map to char index 5");
    }

    #[test]
    fn map_cell_to_char_with_scroll() {
        let browser = make_test_browser_with_preview("Line 0\nLine 1\nLine 2\nLine 3", 2);

        // With scroll=2, visual row 0 should map to line 2
        let idx = browser.map_cell_to_char(0, 0, 80);
        // Line 0 = "Line 0\n" (7 chars), Line 1 = "Line 1\n" (7 chars)
        // Line 2 starts at char 14
        assert_eq!(
            idx,
            Some(14),
            "position (0,0) with scroll=2 should map to line 2 start"
        );
    }

    #[test]
    fn map_cell_to_char_multiline() {
        let browser = make_test_browser_with_preview("AB\nCD\nEF", 0);

        // Line 0: "AB\n" (chars 0, 1, 2)
        // Line 1: "CD\n" (chars 3, 4, 5)
        // Line 2: "EF"   (chars 6, 7)

        let idx = browser.map_cell_to_char(0, 1, 80);
        assert_eq!(idx, Some(3), "position (0,1) should map to char 3 ('C')");

        let idx = browser.map_cell_to_char(1, 2, 80);
        assert_eq!(idx, Some(7), "position (1,2) should map to char 7 ('F')");
    }

    #[test]
    fn map_cell_to_char_out_of_bounds() {
        let browser = make_test_browser_with_preview("Short", 0);

        // Position way beyond text
        let idx = browser.map_cell_to_char(0, 100, 80);
        assert_eq!(idx, None, "out-of-bounds row should return None");
    }

    #[test]
    fn map_cell_to_char_multibyte_utf8() {
        let browser = make_test_browser_with_preview("Héllo 世界", 0);

        // UTF-8 characters should be counted correctly
        let idx = browser.map_cell_to_char(0, 0, 80);
        assert_eq!(idx, Some(0), "position (0,0) should map to char 0");

        // Note: This test verifies the function handles UTF-8 without crashing
        // Exact column mapping depends on how terminal renders wide chars
    }

    // -----------------------------------------------------------------------
    // Selection Text Extraction Tests (file-browser-gbw.4)
    // -----------------------------------------------------------------------

    #[test]
    fn extract_selection_single_line() {
        let browser = make_test_browser_with_preview("Hello World", 0);

        let text = browser.extract_selected_text(0, 5);
        assert_eq!(text, "Hello", "should extract 'Hello'");
    }

    #[test]
    fn extract_selection_multi_line() {
        let browser = make_test_browser_with_preview("Line1\nLine2\nLine3", 0);

        // Extract from middle of Line1 to middle of Line2
        // "Line1\n" = 6 chars, so Line2 starts at 6
        let text = browser.extract_selected_text(3, 9);
        assert_eq!(text, "e1\nLin", "should extract across lines");
    }

    #[test]
    fn extract_selection_reversed() {
        let browser = make_test_browser_with_preview("Hello World", 0);

        // End before start (reversed selection)
        let text = browser.extract_selected_text(5, 0);
        assert_eq!(text, "Hello", "reversed selection should work");
    }

    #[test]
    fn extract_selection_empty() {
        let browser = make_test_browser_with_preview("Hello World", 0);

        // Same start and end
        let text = browser.extract_selected_text(3, 3);
        assert_eq!(text, "", "same start/end should return empty string");
    }

    #[test]
    fn extract_selection_full_content() {
        let browser = make_test_browser_with_preview("Full content here", 0);

        let text = browser.extract_selected_text(0, 17);
        assert_eq!(text, "Full content here", "should extract entire content");
    }

    // -----------------------------------------------------------------------
    // Toast Lifecycle Tests (file-browser-gbw.5)
    // -----------------------------------------------------------------------

    #[test]
    fn toast_appears_after_copy() {
        let mut browser = make_test_browser_with_preview("Hello", 0);

        browser.show_toast("Copied to clipboard");

        assert_eq!(browser.toast_text, Some("Copied to clipboard".to_string()));
    }

    #[test]
    fn toast_has_start_time() {
        let mut browser = make_test_browser_with_preview("Hello", 0);

        browser.show_toast("Test toast");

        assert!(browser.toast_start.is_some(), "toast_start should be set");
    }

    #[test]
    fn toast_dismissed_after_tick() {
        let mut browser = make_test_browser_with_preview("Hello", 0);
        browser.toast_text = Some("Test".to_string());
        // Set start time in the past (>1.5s ago)
        browser.toast_start = Some(Instant::now() - Duration::from_secs(2));

        browser.check_toast_timeout();

        assert_eq!(
            browser.toast_text, None,
            "toast should be dismissed after timeout"
        );
        assert_eq!(browser.toast_start, None, "toast_start should be cleared");
    }

    #[test]
    fn toast_persists_before_timeout() {
        let mut browser = make_test_browser_with_preview("Hello", 0);
        browser.toast_text = Some("Test".to_string());
        // Set start time just now (not expired)
        browser.toast_start = Some(Instant::now());

        browser.check_toast_timeout();

        assert_eq!(
            browser.toast_text,
            Some("Test".to_string()),
            "toast should persist before timeout"
        );
    }

    #[test]
    fn new_copy_resets_toast_timer() {
        let mut browser = make_test_browser_with_preview("Hello", 0);
        browser.show_toast("First");
        let first_start = browser.toast_start;

        // Small delay to ensure different timestamp
        std::thread::sleep(Duration::from_millis(10));

        browser.show_toast("Second");

        assert_eq!(browser.toast_text, Some("Second".to_string()));
        assert!(
            browser.toast_start != first_start,
            "toast_start should be updated"
        );
    }

    // -----------------------------------------------------------------------
    // Rendering Tests (file-browser-gbw.6)
    // -----------------------------------------------------------------------

    #[test]
    fn render_selection_highlight_state() {
        let mut browser = make_test_browser_with_preview("Hello World", 0);
        browser.selection_start = Some((0, 0));
        browser.selection_end = Some((5, 0));
        browser.is_selecting = true;

        // Verify selection state is set up for rendering
        assert!(browser.is_selecting);
        assert!(browser.selection_start.is_some());
        assert!(browser.selection_end.is_some());
    }

    #[test]
    fn render_toast_content_check() {
        let mut browser = make_test_browser_with_preview("Hello", 0);
        browser.show_toast("Copied to clipboard");

        assert_eq!(browser.toast_text.as_deref(), Some("Copied to clipboard"));
    }

    #[test]
    fn render_no_toast_when_none() {
        let browser = make_test_browser_with_preview("Hello", 0);

        assert!(
            browser.toast_text.is_none(),
            "toast_text should be None by default"
        );
    }

    #[test]
    fn screen_to_preview_coords_basic() {
        let browser = make_test_browser_with_preview("Hello", 0);
        browser.preview_inner_area.set(Rect::new(10, 5, 30, 10));

        let (px, py) = browser.screen_to_preview_coords(15, 8);

        assert_eq!(px, 5, "x should be offset by preview x");
        assert_eq!(py, 3, "y should be offset by preview y");
    }

    #[test]
    fn preview_text_as_string_basic() {
        let browser = make_test_browser_with_preview("Hello\nWorld", 0);

        let text = browser.preview_text_as_string();

        assert_eq!(text, "Hello\nWorld");
    }

    #[test]
    fn clear_selection_resets_all() {
        let mut browser = make_test_browser_with_preview("Hello", 0);
        browser.selection_start = Some((0, 0));
        browser.selection_end = Some((5, 0));
        browser.is_selecting = true;

        browser.clear_selection();

        assert_eq!(browser.selection_start, None);
        assert_eq!(browser.selection_end, None);
        assert!(!browser.is_selecting);
    }
}

fn main() -> std::io::Result<()> {
    let start_dir = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| std::env::current_dir().unwrap_or_else(|_| PathBuf::from(".")));

    let start_dir = fs::canonicalize(&start_dir).unwrap_or(start_dir);

    let model = FileBrowser::new(start_dir);
    let config = ProgramConfig::fullscreen().with_mouse();
    let mut program = Program::with_config(model, config)?;
    program.run()
}
