# file-browser

A terminal file browser built with [FrankenTUI](https://github.com/Dicklesworthstone/frankentui) featuring mouse support and markdown rendering.

![file-browser screenshot](assets/screenshot.png)

## Features

- **Split-pane layout** -- directory tree on the left, file preview on the right
- **Mouse support** -- click to select/expand tree nodes, scroll wheel navigation
- **Markdown rendering** -- `.md` files rendered with styled headings, bold, italic, code blocks, lists, links, and tables via `pulldown-cmark`
- **Plain text preview** -- non-markdown files shown with word wrapping
- **Lazy directory loading** -- subdirectories are read only when expanded
- **Scrollbar** -- vertical scrollbar in the preview pane when content overflows
- **Status bar** -- displays filename, size, and keyboard shortcuts

## Install

Requires Rust nightly (FrankenTUI uses edition 2024).

```bash
git clone <this-repo>
cd file-browser
cargo build --release
```

The binary is at `target/release/file-browser`.

## Usage

```bash
# Browse the current directory
./target/release/file-browser

# Browse a specific directory
./target/release/file-browser /path/to/dir
```

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Up` / `k` | Navigate up in tree / scroll preview up |
| `Down` / `j` | Navigate down in tree / scroll preview down |
| `Enter` / `Right` / `l` | Expand directory or open file |
| `Backspace` / `Left` / `h` | Collapse directory or switch to tree |
| `Tab` | Switch focus between tree and preview panes |
| `PageUp` / `PageDown` | Jump 10 items or scroll a full page |
| `Home` / `End` | Go to first/last item |
| `q` / `Esc` | Quit |

## Architecture

Built on FrankenTUI's Elm/Bubbletea architecture (`Model` trait):

- **Layout** -- `Flex::horizontal()` splits tree (30%) and preview (70%); `Flex::vertical()` separates the status bar
- **Tree** -- `Tree` widget with `TreeGuides::Rounded` and lazy-loaded `TreeNode` hierarchy
- **Preview** -- `Paragraph` widget with `WrapMode::Word` and `Scrollbar`
- **Markdown** -- `MarkdownRenderer` from `ftui-extras` with panic-safe fallback for complex LaTeX math
- **Mouse** -- `ProgramConfig::fullscreen().with_mouse()` enables full mouse capture; tree uses `handle_mouse()` hit testing

## Dependencies

| Crate | Purpose |
|-------|---------|
| `ftui` | FrankenTUI public facade |
| `ftui-extras` | Markdown rendering (`markdown` feature) and syntax highlighting (`syntax` feature) |

## License

MIT
