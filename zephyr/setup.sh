#!/bin/bash
# nano-ros Zephyr Workspace Setup
#
# This script creates a Zephyr workspace with nano-ros integrated using
# west manifest. It installs all dependencies including:
#   - Python tools (west, etc.)
#   - Zephyr SDK (cross-compilers)
#   - Zephyr RTOS and modules
#   - zephyr-lang-rust for Rust support
#
# Prerequisites (install manually):
#   - Python 3.8+, pip
#   - cmake, ninja-build
#   - Rust toolchain (rustup)
#
# Usage:
#   ./zephyr/setup.sh [OPTIONS]
#
# Options:
#   --workspace DIR    Workspace directory (default: ~/nano-ros-workspace)
#   --sdk-dir DIR      SDK install directory (default: ~/.local)
#   --force            Overwrite existing workspace
#   --skip-sdk         Skip SDK installation (if already installed)
#
# Example:
#   cd /path/to/nano-ros
#   ./zephyr/setup.sh
#   source ~/nano-ros-workspace/env.sh
#   west build -b native_sim/native/64 nano-ros/examples/zephyr-talker-rs

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NANO_ROS_ROOT="$(dirname "$SCRIPT_DIR")"
DEFAULT_WORKSPACE="$HOME/nano-ros-workspace"
DEFAULT_SDK_DIR="$HOME/.local"
ZEPHYR_SDK_VERSION="0.16.8"

# Parse arguments
WORKSPACE_DIR="$DEFAULT_WORKSPACE"
SDK_DIR="$DEFAULT_SDK_DIR"
FORCE=false
SKIP_SDK=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --workspace)
            WORKSPACE_DIR="$2"
            shift 2
            ;;
        --sdk-dir)
            SDK_DIR="$2"
            shift 2
            ;;
        --force|-f)
            FORCE=true
            shift
            ;;
        --skip-sdk)
            SKIP_SDK=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Create a nano-ros Zephyr workspace"
            echo ""
            echo "Options:"
            echo "  --workspace DIR    Workspace directory (default: ~/nano-ros-workspace)"
            echo "  --sdk-dir DIR      SDK install directory (default: ~/.local)"
            echo "  --force, -f        Overwrite existing workspace"
            echo "  --skip-sdk         Skip SDK installation"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[OK]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

echo ""
echo "========================================"
echo "  nano-ros Zephyr Workspace Setup"
echo "========================================"
echo ""
log_info "Workspace: $WORKSPACE_DIR"
log_info "SDK directory: $SDK_DIR"
log_info "nano-ros: $NANO_ROS_ROOT"
echo ""

# =============================================================================
# Check Prerequisites
# =============================================================================

log_info "Checking prerequisites..."

check_command() {
    if command -v "$1" &> /dev/null; then
        log_success "$1 found"
        return 0
    else
        log_error "$1 not found"
        return 1
    fi
}

MISSING=0
check_command python3 || MISSING=1
check_command pip3 || MISSING=1
check_command cmake || MISSING=1
check_command git || MISSING=1
check_command ninja || { log_warn "ninja not found, trying to install..."; sudo apt-get install -y ninja-build 2>/dev/null || MISSING=1; }
check_command wget || { log_warn "wget not found, trying to install..."; sudo apt-get install -y wget 2>/dev/null || MISSING=1; }
check_command rustc || MISSING=1
check_command cargo || MISSING=1

if [ $MISSING -eq 1 ]; then
    echo ""
    log_error "Missing prerequisites. Please install:"
    echo "  sudo apt install python3 python3-pip python3-venv cmake ninja-build wget git"
    echo "  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh"
    exit 1
fi

# =============================================================================
# Install Python Tools
# =============================================================================

log_info "Installing Python tools..."

pip3 install --user --upgrade pip
pip3 install --user west pyelftools

export PATH="$HOME/.local/bin:$PATH"

if command -v west &> /dev/null; then
    log_success "west installed: $(west --version)"
else
    log_error "west installation failed"
    exit 1
fi

# =============================================================================
# Install Rust Embedded Targets
# =============================================================================

log_info "Installing Rust embedded targets..."

rustup target add thumbv7m-none-eabi 2>/dev/null || true
rustup target add thumbv7em-none-eabi 2>/dev/null || true
rustup target add thumbv7em-none-eabihf 2>/dev/null || true
rustup target add x86_64-unknown-none 2>/dev/null || true

log_success "Rust embedded targets ready"

# =============================================================================
# Install Zephyr SDK
# =============================================================================

SDK_PATH="$SDK_DIR/zephyr-sdk-$ZEPHYR_SDK_VERSION"

if [ "$SKIP_SDK" = true ]; then
    log_info "Skipping SDK installation (--skip-sdk)"
elif [ -d "$SDK_PATH" ] && [ -f "$SDK_PATH/setup.sh" ]; then
    log_info "Zephyr SDK already installed at $SDK_PATH"
else
    log_info "Installing Zephyr SDK $ZEPHYR_SDK_VERSION..."

    mkdir -p "$SDK_DIR"
    cd "$SDK_DIR"

    SDK_TARBALL="zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64.tar.xz"
    SDK_URL="https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/${SDK_TARBALL}"

    if [ ! -f "$SDK_TARBALL" ]; then
        log_info "Downloading Zephyr SDK..."
        wget -q --show-progress "$SDK_URL"
    fi

    log_info "Extracting SDK..."
    tar xf "$SDK_TARBALL"

    log_info "Running SDK setup..."
    cd "$SDK_PATH"
    ./setup.sh -t x86_64-zephyr-elf -t arm-zephyr-eabi -h -c

    rm -f "$SDK_DIR/$SDK_TARBALL"
    log_success "Zephyr SDK installed"
fi

export ZEPHYR_SDK_INSTALL_DIR="$SDK_PATH"

# =============================================================================
# Helper Functions
# =============================================================================

create_env_script() {
    log_info "Creating environment script..."
    cat > "$WORKSPACE_DIR/env.sh" << ENVEOF
#!/bin/bash
# nano-ros Zephyr Environment
# Usage: source env.sh

WORKSPACE="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"

# Zephyr environment
source "\$WORKSPACE/zephyr/zephyr-env.sh"

# Zephyr SDK
export ZEPHYR_SDK_INSTALL_DIR="$SDK_PATH"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr

# nano-ros paths
export NANO_ROS_ROOT="\$WORKSPACE/nano-ros"

# Local bin
export PATH="\$HOME/.local/bin:\$PATH"

echo "nano-ros environment ready"
echo "  ZEPHYR_BASE: \$ZEPHYR_BASE"
echo "  ZEPHYR_SDK: $SDK_PATH"
echo "  NANO_ROS_ROOT: \$NANO_ROS_ROOT"
echo ""
echo "Build example:"
echo "  west build -b native_sim/native/64 nano-ros/examples/zephyr-talker-rs"
ENVEOF
    chmod +x "$WORKSPACE_DIR/env.sh"
}

# =============================================================================
# Initialize Workspace (using west manifest)
# =============================================================================

if [ -d "$WORKSPACE_DIR" ]; then
    if [ "$FORCE" = true ]; then
        log_warn "Removing existing workspace..."
        rm -rf "$WORKSPACE_DIR"
    elif [ -d "$WORKSPACE_DIR/.west" ]; then
        log_info "Workspace exists, updating..."
        cd "$WORKSPACE_DIR"
        west update
        log_success "Update complete"

        # Regenerate env.sh
        create_env_script

        echo ""
        log_success "Workspace ready!"
        echo ""
        echo "Usage:"
        echo "  source $WORKSPACE_DIR/env.sh"
        echo "  west build -b native_sim/native/64 nano-ros/examples/zephyr-talker-rs"
        exit 0
    else
        log_error "Directory exists but not a west workspace: $WORKSPACE_DIR"
        log_info "Use --force to overwrite"
        exit 1
    fi
fi

# Initialize workspace using a copied manifest
# west init -l creates .west in the parent of the manifest repo,
# so we create a manifest directory inside the workspace with west.yml,
# then replace it with a symlink to nano-ros after initialization.
log_info "Initializing workspace..."
mkdir -p "$WORKSPACE_DIR"
cd "$WORKSPACE_DIR"

# Create a manifest directory with a copy of west.yml
# (west init -l follows symlinks, so we must copy)
mkdir -p "$WORKSPACE_DIR/nano-ros"
cp "$NANO_ROS_ROOT/west.yml" "$WORKSPACE_DIR/nano-ros/west.yml"

# Initialize west from the manifest directory
west init -l "$WORKSPACE_DIR/nano-ros"

# Now replace the manifest directory with a symlink to the real nano-ros
rm -rf "$WORKSPACE_DIR/nano-ros"
ln -sf "$NANO_ROS_ROOT" "$WORKSPACE_DIR/nano-ros"

log_info "Fetching Zephyr and modules (this may take a while)..."
west update

# Install Zephyr Python dependencies
log_info "Installing Zephyr Python dependencies..."
pip3 install --user -r zephyr/scripts/requirements.txt

# Create environment script
create_env_script

# =============================================================================
# Summary
# =============================================================================

echo ""
log_success "========================================"
log_success "  Workspace setup complete!"
log_success "========================================"
echo ""
echo "Workspace: $WORKSPACE_DIR"
echo ""
echo "Contents:"
echo "  nano-ros/     - nano-ros project (your code)"
echo "  zephyr/       - Zephyr RTOS v3.7.0"
echo "  modules/      - Zephyr modules (including lang/rust)"
echo ""
echo "Next steps:"
echo ""
echo "  1. Source the environment:"
echo "     source $WORKSPACE_DIR/env.sh"
echo ""
echo "  2. Build an example:"
echo "     west build -b native_sim/native/64 nano-ros/examples/zephyr-talker-rs"
echo ""
echo "  3. Run:"
echo "     ./build/zephyr/zephyr.exe"
echo ""
