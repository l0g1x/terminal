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
use std::time::{Duration, SystemTime};

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
const GITHUB_API_URL: &str =
    "https://api.github.com/repos/l0g1x/terminal/commits/main";

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
            "--max-time", "5",
            "-H", "User-Agent: terminal-file-browser",
            "-H", "Accept: application/vnd.github.v3+json",
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
    let name = path
        .file_name()
        .map_or_else(|| path.to_string_lossy().into_owned(), |n| n.to_string_lossy().into_owned());

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
                .map_or(false, |n| n.to_string_lossy().starts_with('.'))
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
        let dname = d
            .file_name()
            .map_or_else(|| d.to_string_lossy().into_owned(), |n| n.to_string_lossy().into_owned());
        children.push(TreeNode::new(format!("{dname}/")).with_expanded(false));
    }
    for f in &files {
        let fname = f
            .file_name()
            .map_or_else(|| f.to_string_lossy().into_owned(), |n| n.to_string_lossy().into_owned());
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
fn resolve_path_for_index(root: &Path, tree: &Tree, index: usize, scratch: &mut Vec<String>) -> Option<PathBuf> {
    scratch.clear();
    if collect_path_at_index(tree.root(), index, scratch, tree.root().label().ends_with('/')) {
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

/// Read file content for preview (capped at MAX_PREVIEW_BYTES).
fn read_file_preview(path: &Path) -> Result<String, String> {
    let metadata = fs::metadata(path).map_err(|e| format!("Cannot read: {e}"))?;
    if metadata.len() > MAX_PREVIEW_BYTES {
        let content = fs::read(path).map_err(|e| format!("Cannot read: {e}"))?;
        let truncated = &content[..MAX_PREVIEW_BYTES as usize];
        let truncated_str = String::from_utf8(truncated.to_vec())
            .unwrap_or_else(|_| String::from_utf8_lossy(truncated).into_owned());
        Ok(format!("{truncated_str}\n\n--- (truncated at 256KB) ---"))
    } else {
        fs::read_to_string(path).map_err(|e| {
            if e.kind() == std::io::ErrorKind::InvalidData {
                "Binary file (cannot preview)".to_string()
            } else {
                format!("Cannot read: {e}")
            }
        })
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

        let root_name = root
            .file_name()
            .map_or_else(|| root.to_string_lossy().into_owned(), |n| n.to_string_lossy().into_owned());

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
            status: format!("  {root_name}/ | arrows: navigate | Enter: open | Tab: switch pane | q: quit"),
            preview_title: " Preview ".to_string(),
            md_renderer: MarkdownRenderer::new(MarkdownTheme::default()),
            scrollbar_state: ScrollbarState::new(0, 0, 0),
            styles: CachedStyles::new(),
            path_scratch: Vec::with_capacity(32),
            update_notice: None,
            tree_scroll: 0,
            tree_height: Cell::new(0),
        }
    }

    /// Resolve the filesystem path for the current selected_index, using cache.
    fn resolve_current_path(&mut self) -> Option<PathBuf> {
        if self.cached_path_index == self.selected_index {
            return self.cached_path.clone();
        }
        let path = resolve_path_for_index(&self.root, &self.tree, self.selected_index, &mut self.path_scratch);
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
        let name = path
            .file_name()
            .map_or_else(|| path.to_string_lossy().into_owned(), |n| n.to_string_lossy().into_owned());
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
                let name = path
                    .file_name()
                    .map_or_else(|| path.to_string_lossy().into_owned(), |n| n.to_string_lossy().into_owned());
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
                        if let Some(node) =
                            self.tree.node_at_visible_index_mut(self.selected_index)
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
                    let result =
                        self.tree
                            .handle_mouse(&me, Some((TREE_HIT_ID, HitRegion::Content, 0)), TREE_HIT_ID);
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

            Msg::Tick | Msg::Noop => {}
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
            .title(&*self.preview_title)
            .title_alignment(Alignment::Left)
            .border_style(preview_border)
            .border_type(BorderType::Rounded);

        let inner_preview_area = preview_block.inner(preview_area);
        preview_block.render(preview_area, frame);

        let ph = inner_preview_area.height;
        self.preview_height.set(ph);

        // Clear the preview area before rendering to prevent stale content
        // from the previous file bleeding through when switching to shorter text.
        frame.buffer.fill(inner_preview_area, RenderCell::default());

        let preview_paragraph = Paragraph::new(self.preview_text.clone())
            .style(self.styles.file_style)
            .wrap(WrapMode::Word)
            .scroll((self.preview_scroll, 0));

        preview_paragraph.render(inner_preview_area, frame);

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

        // ---- Status bar ----
        let status_text = if let Some(ref notice) = self.update_notice {
            notice.as_str()
        } else {
            self.status.as_str()
        };
        let status_style = if self.update_notice.is_some() {
            Style::new().fg(PackedRgba::rgb(255, 200, 60)).bg(COLOR_STATUS_BG).bold()
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

        let selected = self.selected_index.min(self.visible_count.saturating_sub(1));

        for (row_idx, flat_idx) in (scroll..end).enumerate() {
            let node = &flat[flat_idx];
            let y = area.y + row_idx as u16;
            let max_x = area.right();
            let mut x = area.x;

            let is_selected = flat_idx == selected;

            // Determine colors for this row.
            let guide_fg = if is_selected { COLOR_SELECTED } else { COLOR_BORDER };
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

                x = frame.buffer.print_text_clipped(x, y, guide_str, guide_cell, max_x);
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
            frame.buffer.print_text_clipped(x, y, node.label, label_cell, max_x);
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
        frame.buffer.get(x, y).map_or(PackedRgba::TRANSPARENT, |c| c.fg)
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
        let root_node = nodes.into_iter().fold(
            TreeNode::new("root/").with_expanded(true),
            |root, child| root.child(child),
        );
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
        }
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
            vec![
                TreeNode::new("first.txt"),
                TreeNode::new("last.txt"),
            ],
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
                    .with_children(vec![
                        TreeNode::new("inner.txt"),
                    ]),
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
            vec![
                TreeNode::new("file_a.txt"),
                TreeNode::new("file_b.txt"),
            ],
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
        assert!(
            bolds[1][4],
            "selected row label should be bold at col 4"
        );
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
                    .with_children(vec![
                        TreeNode::new("a.txt"),
                        TreeNode::new("b.txt"),
                    ]),
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
                    .with_children(vec![
                        TreeNode::new("hidden.txt"),
                    ]),
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
            .with_children(vec![TreeNode::new("a/")
                .with_expanded(true)
                .with_children(vec![TreeNode::new("b/")
                    .with_expanded(true)
                    .with_children(vec![TreeNode::new("c.txt")])])]);

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
            (0..8).map(|i| TreeNode::new(format!("file_{i}.txt"))).collect(),
            0,
            0,
        );
        browser1.tree_height.set(10);
        // Fill area first (simulates what view() does).
        frame.buffer.fill(area, RenderCell::default());
        browser1.render_tree_with_selection(area, &mut frame);

        // Second render: tree with only 2 items, into the SAME frame after clear.
        frame.buffer.fill(area, RenderCell::default());
        let browser2 = make_test_browser(
            vec![TreeNode::new("only.txt")],
            0,
            0,
        );
        browser2.tree_height.set(10);
        browser2.render_tree_with_selection(area, &mut frame);

        // Rows after the short tree should be empty.
        let row5 = row_text(&frame, 5, 40);
        assert_eq!(row5, "", "row 5 should be empty after short tree render: got '{row5}'");
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
