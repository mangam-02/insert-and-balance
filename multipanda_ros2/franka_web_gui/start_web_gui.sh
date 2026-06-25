#!/usr/bin/env bash
# Run inside the multipanda container.
# Starts the web GUI in mock/dummy mode — no ROS, no sudo required.
set -e

GUI_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NVM_DIR="$HOME/.nvm"

# ── 1. Install nvm + Node.js 20 if missing (no sudo needed) ─────────────────
if ! command -v node &>/dev/null; then
    echo "Installing nvm + Node.js 20 (no sudo required)..."
    curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
    export NVM_DIR="$HOME/.nvm"
    # shellcheck source=/dev/null
    source "$NVM_DIR/nvm.sh"
    nvm install 20
    nvm use 20
else
    # nvm may not be sourced yet in this shell
    [ -s "$NVM_DIR/nvm.sh" ] && source "$NVM_DIR/nvm.sh"
fi

echo "Node $(node --version) / npm $(npm --version)"

# ── 2. Install npm deps & start Vite dev server (mock mode) ──────────────────
cd "$GUI_DIR/frontend"
npm install
npm run dev
