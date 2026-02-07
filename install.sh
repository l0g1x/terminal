#!/usr/bin/env bash
set -euo pipefail

# terminal - FrankenTUI file browser installer
# Usage: curl -sSf https://raw.githubusercontent.com/l0g1x/terminal/main/install.sh | bash
#
# 1. Detects your OS and architecture
# 2. Downloads a pre-built binary from the latest GitHub release
# 3. Falls back to building from source if no binary is available

REPO_OWNER="l0g1x"
REPO_NAME="terminal"
REPO_URL="https://github.com/$REPO_OWNER/$REPO_NAME"
BIN_NAME="terminal"
INSTALL_DIR="${CARGO_HOME:-$HOME/.cargo}/bin"

info()  { printf '\033[1;34m>>>\033[0m %s\n' "$*"; }
ok()    { printf '\033[1;32m>>>\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m>>>\033[0m %s\n' "$*"; }
err()   { printf '\033[1;31m>>>\033[0m %s\n' "$*" >&2; }

WORKDIR=""
cleanup() {
    if [ -n "$WORKDIR" ] && [ -d "$WORKDIR" ]; then
        rm -rf "$WORKDIR"
    fi
}
trap cleanup EXIT

# ---------------------------------------------------------------------------
# Detect platform
# ---------------------------------------------------------------------------

detect_platform() {
    local os arch

    case "$(uname -s)" in
        Linux*)  os="linux" ;;
        Darwin*) os="darwin" ;;
        *)       os="unknown" ;;
    esac

    case "$(uname -m)" in
        x86_64|amd64)  arch="x86_64" ;;
        aarch64|arm64) arch="aarch64" ;;
        *)             arch="unknown" ;;
    esac

    echo "${os}-${arch}"
}

PLATFORM="$(detect_platform)"
ARTIFACT="terminal-${PLATFORM}"
info "Detected platform: $PLATFORM"

# ---------------------------------------------------------------------------
# Try downloading a pre-built binary (fast path)
# ---------------------------------------------------------------------------

try_download() {
    # Get the latest release tag from the GitHub API
    local release_url="https://api.github.com/repos/$REPO_OWNER/$REPO_NAME/releases/latest"
    local download_url

    info "Checking for pre-built binary..."

    # Fetch the download URL for our platform's artifact
    download_url=$(
        curl -sSf --max-time 10 \
            -H "Accept: application/vnd.github.v3+json" \
            "$release_url" 2>/dev/null \
        | grep -o "\"browser_download_url\":[[:space:]]*\"[^\"]*${ARTIFACT}[^\"]*\"" \
        | head -1 \
        | sed 's/.*"browser_download_url":[[:space:]]*"//;s/"//'
    ) || return 1

    if [ -z "$download_url" ]; then
        return 1
    fi

    info "Downloading $ARTIFACT..."
    WORKDIR="$(mktemp -d)"
    curl -sSfL --max-time 60 -o "$WORKDIR/$BIN_NAME" "$download_url" || return 1

    # Sanity check: must be a real binary, not an HTML error page
    if [ ! -s "$WORKDIR/$BIN_NAME" ]; then
        return 1
    fi

    return 0
}

# ---------------------------------------------------------------------------
# Build from source (fallback)
# ---------------------------------------------------------------------------

build_from_source() {
    warn "No pre-built binary for $PLATFORM -- building from source."

    # Rust
    if ! command -v cargo &>/dev/null; then
        info "Rust not found -- installing via rustup..."
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain nightly
        # shellcheck source=/dev/null
        source "${CARGO_HOME:-$HOME/.cargo}/env"
    elif ! rustup toolchain list 2>/dev/null | grep -q nightly; then
        info "Installing Rust nightly toolchain..."
        rustup toolchain install nightly
    fi

    # Git
    if ! command -v git &>/dev/null; then
        err "git is required but not found. Please install git first."
        exit 1
    fi

    WORKDIR="$(mktemp -d)"
    info "Cloning $REPO_URL..."
    git clone --depth 1 "$REPO_URL.git" "$WORKDIR/repo"

    info "Building release binary (this may take a couple of minutes)..."
    cargo +nightly build --release --manifest-path "$WORKDIR/repo/Cargo.toml"

    cp "$WORKDIR/repo/target/release/$BIN_NAME" "$WORKDIR/$BIN_NAME"
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if ! try_download; then
    build_from_source
fi

# Install
mkdir -p "$INSTALL_DIR"
cp "$WORKDIR/$BIN_NAME" "$INSTALL_DIR/$BIN_NAME"
chmod +x "$INSTALL_DIR/$BIN_NAME"

# Ensure INSTALL_DIR is in PATH (persist to shell profile if needed)
if ! echo "$PATH" | tr ':' '\n' | grep -qx "$INSTALL_DIR"; then
    EXPORT_LINE="export PATH=\"$INSTALL_DIR:\$PATH\""

    # Find the right shell profile to update
    SHELL_PROFILE=""
    case "$(basename "${SHELL:-/bin/bash}")" in
        zsh)  SHELL_PROFILE="$HOME/.zshrc" ;;
        fish) ;;  # fish uses a different mechanism; skip
        *)    SHELL_PROFILE="$HOME/.bashrc" ;;
    esac

    # Append if not already present
    if [ -n "$SHELL_PROFILE" ] && [ -f "$SHELL_PROFILE" ]; then
        if ! grep -qF "$INSTALL_DIR" "$SHELL_PROFILE" 2>/dev/null; then
            printf '\n# Added by terminal installer\n%s\n' "$EXPORT_LINE" >> "$SHELL_PROFILE"
            info "Added $INSTALL_DIR to PATH in $SHELL_PROFILE"
        fi
    elif [ -n "$SHELL_PROFILE" ]; then
        printf '# Added by terminal installer\n%s\n' "$EXPORT_LINE" > "$SHELL_PROFILE"
        info "Created $SHELL_PROFILE with PATH entry"
    fi

    # Also export for the current invocation context
    export PATH="$INSTALL_DIR:$PATH"
    ok "Installed! Open a new shell or run:  source $SHELL_PROFILE"
else
    ok "Installed! Run it with:  terminal"
fi
