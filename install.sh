#!/usr/bin/env bash
set -euo pipefail

# terminal - FrankenTUI file browser installer
# Usage: curl -sSf https://raw.githubusercontent.com/l0g1x/terminal/main/install.sh | bash

REPO="https://github.com/l0g1x/terminal.git"
BIN_NAME="terminal"
INSTALL_DIR="${CARGO_HOME:-$HOME/.cargo}/bin"

info()  { printf '\033[1;34m>>>\033[0m %s\n' "$*"; }
ok()    { printf '\033[1;32m>>>\033[0m %s\n' "$*"; }
err()   { printf '\033[1;31m>>>\033[0m %s\n' "$*" >&2; }

WORKDIR=""
cleanup() {
    if [ -n "$WORKDIR" ] && [ -d "$WORKDIR" ]; then
        rm -rf "$WORKDIR"
    fi
}
trap cleanup EXIT

# --- Check / install Rust ---------------------------------------------------

if ! command -v cargo &>/dev/null; then
    info "Rust not found -- installing via rustup..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain nightly
    # shellcheck source=/dev/null
    source "${CARGO_HOME:-$HOME/.cargo}/env"
elif ! rustup toolchain list 2>/dev/null | grep -q nightly; then
    info "Installing Rust nightly toolchain..."
    rustup toolchain install nightly
fi

# --- Check for git -----------------------------------------------------------

if ! command -v git &>/dev/null; then
    err "git is required but not found. Please install git first."
    exit 1
fi

# --- Clone & build -----------------------------------------------------------

WORKDIR="$(mktemp -d)"
info "Cloning $REPO..."
git clone --depth 1 "$REPO" "$WORKDIR/terminal"

info "Building release binary (this may take a couple of minutes)..."
cargo +nightly build --release --manifest-path "$WORKDIR/terminal/Cargo.toml"

# --- Install -----------------------------------------------------------------

mkdir -p "$INSTALL_DIR"
cp "$WORKDIR/terminal/target/release/$BIN_NAME" "$INSTALL_DIR/$BIN_NAME"
chmod +x "$INSTALL_DIR/$BIN_NAME"

# --- Verify ------------------------------------------------------------------

if echo "$PATH" | tr ':' '\n' | grep -qx "$INSTALL_DIR"; then
    ok "Installed! Run it with: terminal"
else
    ok "Installed to $INSTALL_DIR/$BIN_NAME"
    info "Add to PATH if not already present:"
    info "  export PATH=\"$INSTALL_DIR:\$PATH\""
fi
