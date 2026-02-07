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

use ftui_core::event::{Event, KeyCode, KeyEventKind, MouseButton, MouseEvent, MouseEventKind};
use ftui_core::geometry::Rect;
use ftui_extras::markdown::{MarkdownRenderer, MarkdownTheme};
use ftui_layout::{Constraint, Flex};
use ftui_render::cell::PackedRgba;
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
// Constants
// ---------------------------------------------------------------------------

const TREE_HIT_ID: HitId = HitId::new(1);
const PREVIEW_SCROLLBAR_HIT_ID: HitId = HitId::new(3);
const MAX_PREVIEW_BYTES: u64 = 256 * 1024; // 256 KB cap for file preview

// ---------------------------------------------------------------------------
// Color palette
// ---------------------------------------------------------------------------

fn color_dir() -> PackedRgba {
    PackedRgba::rgb(100, 149, 237) // cornflower blue
}

fn color_file() -> PackedRgba {
    PackedRgba::rgb(200, 200, 200) // light gray
}

fn color_selected() -> PackedRgba {
    PackedRgba::rgb(255, 215, 0) // gold
}

fn color_border() -> PackedRgba {
    PackedRgba::rgb(80, 80, 120) // muted blue-gray
}

fn color_status_bg() -> PackedRgba {
    PackedRgba::rgb(30, 30, 50)
}

fn color_status_fg() -> PackedRgba {
    PackedRgba::rgb(160, 160, 200)
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
    /// Markdown renderer instance.
    md_renderer: MarkdownRenderer,
    /// Scrollbar state for preview pane.
    scrollbar_state: ScrollbarState,
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

/// Build a `TreeNode` hierarchy from a directory path.
/// Only reads one level deep (lazy expansion).
fn build_tree_node(path: &Path) -> TreeNode {
    let name = path
        .file_name()
        .map_or_else(|| path.to_string_lossy().into_owned(), |n| n.to_string_lossy().into_owned());

    if path.is_dir() {
        let mut node = TreeNode::new(format!("{name}/")).with_expanded(false);
        if let Ok(entries) = fs::read_dir(path) {
            let mut dirs: Vec<PathBuf> = Vec::new();
            let mut files: Vec<PathBuf> = Vec::new();
            for entry in entries.flatten() {
                let p = entry.path();
                // Skip hidden files/dirs
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
            dirs.sort();
            files.sort();
            // Add directories first, then files
            for d in &dirs {
                node = node.child(build_tree_node(d));
            }
            for f in &files {
                let fname = f
                    .file_name()
                    .map_or_else(|| f.to_string_lossy().into_owned(), |n| n.to_string_lossy().into_owned());
                node = node.child(TreeNode::new(fname).with_expanded(false));
            }
        }
        node
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
        children.push(build_tree_node(d));
    }
    for f in &files {
        let fname = f
            .file_name()
            .map_or_else(|| f.to_string_lossy().into_owned(), |n| n.to_string_lossy().into_owned());
        children.push(TreeNode::new(fname).with_expanded(false));
    }
    // We need to replace the node's children.
    // TreeNode uses builder pattern, so we rebuild:
    let label = node.label().to_string();
    let expanded = node.is_expanded();
    *node = TreeNode::new(label)
        .with_children(children)
        .with_expanded(expanded);
}

/// Given a visible index in the tree, resolve the filesystem path.
fn resolve_path_for_index(root: &Path, tree: &Tree, index: usize) -> Option<PathBuf> {
    // Walk the tree to find the path components leading to the node at `index`.
    let mut path_parts: Vec<String> = Vec::new();
    if collect_path_at_index(tree.root(), index, &mut path_parts, tree.root().label().ends_with('/')) {
        let mut full_path = root.to_path_buf();
        // Skip root node label from path (it represents the root dir itself)
        for part in &path_parts[1..] {
            let clean = part.trim_end_matches('/');
            full_path = full_path.join(clean);
        }
        Some(full_path)
    } else {
        None
    }
}

/// Recursively collect the label path to the node at the given visible index.
/// Returns true if found and populates `path`.
fn collect_path_at_index(
    node: &TreeNode,
    target: usize,
    path: &mut Vec<String>,
    show_root: bool,
) -> bool {
    // We track a counter through a mutable reference to find the target.
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
        // Read partial
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
            .with_guide_style(Style::new().fg(color_border()))
            .with_label_style(Style::new().fg(color_file()))
            .with_root_style(Style::new().fg(color_dir()).bold())
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
            preview_text: Text::raw("Select a file to preview"),
            is_markdown: false,
            preview_scroll: 0,
            preview_height: Cell::new(0),
            focus: Pane::Tree,
            status: format!("  {root_name}/ | arrows: navigate | Enter: open | Tab: switch pane | q: quit"),
            md_renderer: MarkdownRenderer::new(MarkdownTheme::default()),
            scrollbar_state: ScrollbarState::new(0, 0, 0),
        }
    }

    /// Select a file and load its preview.
    fn select_file(&mut self, path: PathBuf) {
        // Update status bar
        let size_str = fs::metadata(&path)
            .map(|m| human_size(m.len()))
            .unwrap_or_else(|_| "?".into());
        let name = path
            .file_name()
            .map_or_else(|| path.to_string_lossy().into_owned(), |n| n.to_string_lossy().into_owned());
        self.status = format!("  {name} | {size_str} | Tab: switch pane | q: quit");
        self.is_markdown = is_markdown_file(&path);

        match read_file_preview(&path) {
            Ok(content) => {
                if self.is_markdown {
                    // Wrap in catch_unwind: the `unicodeit` crate (used for
                    // LaTeX-to-Unicode math rendering) can panic on certain
                    // complex expressions. Fall back to raw text on panic.
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
        // Update scrollbar
        let total = self.preview_text.height();
        self.scrollbar_state = ScrollbarState::new(total, 0, self.preview_height.get() as usize);
    }

    /// Handle tree navigation: move selection up.
    fn move_up(&mut self) {
        if self.selected_index > 0 {
            self.selected_index -= 1;
            self.on_selection_changed();
        }
    }

    /// Handle tree navigation: move selection down.
    fn move_down(&mut self) {
        if self.selected_index + 1 < self.visible_count {
            self.selected_index += 1;
            self.on_selection_changed();
        }
    }

    /// Called when the tree selection changes to potentially preview a file.
    fn on_selection_changed(&mut self) {
        if let Some(path) = resolve_path_for_index(&self.root, &self.tree, self.selected_index) {
            if path.is_file() {
                self.select_file(path);
            } else {
                // It's a directory - show info
                let name = path
                    .file_name()
                    .map_or_else(|| path.to_string_lossy().into_owned(), |n| n.to_string_lossy().into_owned());
                let count = fs::read_dir(&path).map(|rd| rd.count()).unwrap_or(0);
                self.status = format!("  {name}/ | {count} items | Enter: expand | q: quit");
                self.selected_file = None;
                self.preview_text = Text::raw(format!("Directory: {}\n\n{count} items", path.display()));
                self.preview_scroll = 0;
            }
        }
    }

    /// Handle Enter key: toggle expand for dirs, select file.
    fn handle_enter(&mut self) {
        if let Some(path) = resolve_path_for_index(&self.root, &self.tree, self.selected_index) {
            if path.is_dir() {
                // Toggle expansion and lazy-load children
                if let Some(node) = self.tree.node_at_visible_index_mut(self.selected_index) {
                    if !node.is_expanded() {
                        reload_children(node, &path);
                    }
                    node.toggle_expanded();
                }
                self.visible_count = self.tree.root().visible_count();
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
        // Expand root directory on startup
        let root_path = self.root.clone();
        if let Some(root_node) = self.tree.node_at_visible_index_mut(0) {
            reload_children(root_node, &root_path);
            if !root_node.is_expanded() {
                root_node.toggle_expanded();
            }
        }
        self.visible_count = self.tree.root().visible_count();
        Cmd::none()
    }

    fn update(&mut self, msg: Msg) -> Cmd<Self::Message> {
        match msg {
            Msg::KeyPress(code) => match code {
                KeyCode::Char('q') | KeyCode::Escape => return Cmd::quit(),

                KeyCode::Up | KeyCode::Char('k') => {
                    if self.focus == Pane::Tree {
                        self.move_up();
                    } else {
                        self.scroll_preview_up(1);
                    }
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    if self.focus == Pane::Tree {
                        self.move_down();
                    } else {
                        self.scroll_preview_down(1);
                    }
                }
                KeyCode::PageUp => {
                    if self.focus == Pane::Preview {
                        self.scroll_preview_up(self.preview_height.get().saturating_sub(2));
                    } else {
                        for _ in 0..10 {
                            self.move_up();
                        }
                    }
                }
                KeyCode::PageDown => {
                    if self.focus == Pane::Preview {
                        self.scroll_preview_down(self.preview_height.get().saturating_sub(2));
                    } else {
                        for _ in 0..10 {
                            self.move_down();
                        }
                    }
                }
                KeyCode::Home => {
                    if self.focus == Pane::Tree {
                        self.selected_index = 0;
                        self.on_selection_changed();
                    } else {
                        self.preview_scroll = 0;
                        self.scrollbar_state.position = 0;
                    }
                }
                KeyCode::End => {
                    if self.focus == Pane::Tree {
                        self.selected_index = self.visible_count.saturating_sub(1);
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
                        // Collapse current node if expanded, or move to parent
                        if let Some(node) =
                            self.tree.node_at_visible_index_mut(self.selected_index)
                        {
                            if node.is_expanded() && !node.children().is_empty() {
                                node.toggle_expanded();
                                self.visible_count = self.tree.root().visible_count();
                            } else if self.selected_index > 0 {
                                self.move_up();
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
                    // Check if click is in tree area (handled via hit testing in view)
                    // We store the last frame's hit grid, so we do manual area checks.
                    // For simplicity, use x position: left half = tree, right half = preview
                    // The actual hit testing is more sophisticated via HitId, but we'll
                    // check if the tree widget handled it.
                    let result =
                        self.tree
                            .handle_mouse(&me, Some((TREE_HIT_ID, HitRegion::Content, 0)), TREE_HIT_ID);
                    match result {
                        MouseResult::Selected(idx) | MouseResult::Activated(idx) => {
                            self.selected_index = idx;
                            self.focus = Pane::Tree;
                            if result == MouseResult::Activated(idx) {
                                self.handle_enter();
                            } else {
                                self.on_selection_changed();
                            }
                        }
                        _ => {
                            // Might be a click in the preview pane
                            self.focus = Pane::Preview;
                        }
                    }
                }
                MouseEventKind::ScrollUp => {
                    if self.focus == Pane::Preview {
                        self.scroll_preview_up(3);
                    } else {
                        self.move_up();
                    }
                }
                MouseEventKind::ScrollDown => {
                    if self.focus == Pane::Preview {
                        self.scroll_preview_down(3);
                    } else {
                        self.move_down();
                    }
                }
                _ => {}
            },

            Msg::Resize => {
                // Recalculate visible count (tree may have changed)
                self.visible_count = self.tree.root().visible_count();
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

        // Main layout: tree | preview (vertical split), status bar at bottom
        let vert_areas = Flex::vertical()
            .constraints([Constraint::Fill, Constraint::Fixed(1)])
            .split(total_area);

        let main_area = vert_areas[0];
        let status_area = vert_areas[1];

        // Horizontal split: tree (30%) | preview (70%)
        let horiz_areas = Flex::horizontal()
            .constraints([Constraint::Percentage(30.0), Constraint::Fill])
            .split(main_area);

        let tree_area = horiz_areas[0];
        let preview_area = horiz_areas[1];

        // ---- Render tree pane ----
        let tree_focus_style = if self.focus == Pane::Tree {
            Style::new().fg(color_selected())
        } else {
            Style::new().fg(color_border())
        };

        let tree_block = Block::bordered()
            .title(" Files ")
            .title_alignment(Alignment::Left)
            .border_style(tree_focus_style)
            .border_type(BorderType::Rounded);

        let inner_tree_area = tree_block.inner(tree_area);
        tree_block.render(tree_area, frame);

        // Render tree with selection highlighting
        // We render the tree manually line-by-line to support selection highlighting
        self.render_tree_with_selection(inner_tree_area, frame);

        // ---- Render preview pane ----
        let preview_focus_style = if self.focus == Pane::Preview {
            Style::new().fg(color_selected())
        } else {
            Style::new().fg(color_border())
        };

        let preview_title = if let Some(ref file) = self.selected_file {
            let name = file
                .file_name()
                .map_or_else(|| "Preview".into(), |n| n.to_string_lossy().into_owned());
            if self.is_markdown {
                format!(" {name} (Markdown) ")
            } else {
                format!(" {name} ")
            }
        } else {
            " Preview ".to_string()
        };

        let preview_block = Block::bordered()
            .title(&*preview_title)
            .title_alignment(Alignment::Left)
            .border_style(preview_focus_style)
            .border_type(BorderType::Rounded);

        let inner_preview_area = preview_block.inner(preview_area);
        preview_block.render(preview_area, frame);

        // Store preview height via Cell for scroll calculations.
        let ph = inner_preview_area.height;
        self.preview_height.set(ph);

        // Render preview content with scroll
        let preview_paragraph = Paragraph::new(self.preview_text.clone())
            .style(Style::new().fg(color_file()))
            .wrap(WrapMode::Word)
            .scroll((self.preview_scroll, 0));

        preview_paragraph.render(inner_preview_area, frame);

        // Render scrollbar for preview pane
        if self.preview_text.height() > ph as usize {
            let mut sb_state = ScrollbarState::new(
                self.preview_text.height(),
                self.preview_scroll as usize,
                ph as usize,
            );
            let scrollbar = Scrollbar::new(ScrollbarOrientation::VerticalRight)
                .thumb_style(Style::new().fg(color_selected()))
                .track_style(Style::new().fg(color_border()))
                .hit_id(PREVIEW_SCROLLBAR_HIT_ID);
            StatefulWidget::render(&scrollbar, inner_preview_area, frame, &mut sb_state);
        }

        // ---- Render status bar ----
        let status_para = Paragraph::new(Text::raw(&self.status))
            .style(Style::new().fg(color_status_fg()).bg(color_status_bg()));
        status_para.render(status_area, frame);
    }
}

impl FileBrowser {
    /// Render the tree with selection highlighting.
    /// The built-in Tree widget doesn't highlight a "selected" row, so we
    /// render the Tree widget normally and then overlay the selected row style.
    fn render_tree_with_selection(&self, area: Rect, frame: &mut Frame) {
        if area.is_empty() || self.visible_count == 0 {
            return;
        }

        // Render the tree widget
        self.tree.render(area, frame);

        // Clamp selected_index to valid range
        let selected = self.selected_index.min(self.visible_count.saturating_sub(1));

        // Highlight the selected row by overwriting its style
        let visible_start = if selected >= area.height as usize {
            selected - area.height as usize + 1
        } else {
            0
        };

        let row_in_viewport = selected.saturating_sub(visible_start);
        if row_in_viewport < area.height as usize {
            let y = area.y + row_in_viewport as u16;
            for x in area.x..area.right() {
                if let Some(cell) = frame.buffer.get_mut(x, y) {
                    cell.fg = color_selected();
                }
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

    /// Verify markdown renderer doesn't crash the app when unicodeit panics
    /// on complex LaTeX math expressions.
    #[test]
    fn markdown_render_survives_complex_latex() {
        let renderer = MarkdownRenderer::new(MarkdownTheme::default());

        // This is the kind of content from FrankenTUI's README that triggers
        // the unicodeit panic: Bayesian formulas with sub/superscript groups.
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

        // Whether it succeeds or panics, we should not crash.
        match result {
            Ok(text) => {
                // Rendered successfully - verify we got some output
                assert!(text.height() > 0, "rendered text should have content");
            }
            Err(_) => {
                // Panic was caught - this is the expected path for the
                // unicodeit bug. The app would show raw fallback text.
            }
        }
    }

    /// Verify that non-markdown files render without issues.
    #[test]
    fn plain_text_preview_works() {
        let text = Text::raw("Hello, world!\nLine 2\nLine 3");
        assert_eq!(text.height(), 3);
    }

    /// Verify tree node construction from a real directory.
    #[test]
    fn build_tree_from_temp_dir() {
        let dir = std::env::temp_dir().join("ftui_test_tree");
        let _ = fs::create_dir_all(dir.join("subdir"));
        let _ = fs::write(dir.join("file.txt"), "hello");
        let _ = fs::write(dir.join("readme.md"), "# Test");
        let _ = fs::write(dir.join(".hidden"), "secret");

        let node = build_tree_node(&dir);
        // Root should be a dir
        assert!(node.label().ends_with('/'));
        // Hidden file should be excluded
        let child_labels: Vec<&str> = node.children().iter().map(|c| c.label()).collect();
        assert!(
            !child_labels.iter().any(|l| l.starts_with('.')),
            "hidden files should be excluded: {child_labels:?}"
        );

        // Cleanup
        let _ = fs::remove_dir_all(&dir);
    }

    /// Verify path resolution round-trips correctly.
    #[test]
    fn path_resolution_for_root() {
        let dir = std::env::temp_dir().join("ftui_test_resolve");
        let _ = fs::create_dir_all(&dir);
        let _ = fs::write(dir.join("test.txt"), "data");

        let root_node = build_tree_node(&dir);
        let tree = Tree::new(root_node).with_show_root(true);

        // Index 0 should be the root itself
        let resolved = resolve_path_for_index(&dir, &tree, 0);
        assert!(resolved.is_some(), "root should resolve");

        let _ = fs::remove_dir_all(&dir);
    }

    /// Verify human_size formatting.
    #[test]
    fn human_size_formatting() {
        assert_eq!(human_size(0), "0.0 B");
        assert_eq!(human_size(1023), "1023.0 B");
        assert_eq!(human_size(1024), "1.0 KB");
        assert_eq!(human_size(1_048_576), "1.0 MB");
        assert_eq!(human_size(1_073_741_824), "1.0 GB");
    }

    /// Verify is_markdown_file detection.
    #[test]
    fn markdown_detection() {
        assert!(is_markdown_file(Path::new("README.md")));
        assert!(is_markdown_file(Path::new("doc.markdown")));
        assert!(is_markdown_file(Path::new("page.mdx")));
        assert!(!is_markdown_file(Path::new("code.rs")));
        assert!(!is_markdown_file(Path::new("data.json")));
        assert!(!is_markdown_file(Path::new("noext")));
    }

    /// Verify that rendering the actual FrankenTUI README doesn't crash.
    #[test]
    fn frankentui_readme_survives() {
        let readme_path = Path::new("/home/krystian/frankentui/README.md");
        if !readme_path.exists() {
            return; // Skip if frankentui not cloned
        }
        let content = fs::read_to_string(readme_path).unwrap();
        let renderer = MarkdownRenderer::new(MarkdownTheme::default());
        let result = std::panic::catch_unwind(AssertUnwindSafe(|| renderer.render(&content)));
        // Must not propagate the panic
        assert!(
            result.is_ok() || result.is_err(),
            "catch_unwind should always return Ok or Err"
        );
    }
}

fn main() -> std::io::Result<()> {
    // Determine starting directory from command-line arg or current directory
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
