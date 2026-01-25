#!/bin/bash
# nano-ros Zephyr Workspace Setup
#
# This script creates a Zephyr workspace as a sibling to the nano-ros repository.
# It installs all dependencies including:
#   - Python tools (west, etc.)
#   - Zephyr SDK (cross-compilers)
#   - Zephyr RTOS and modules
#   - zephyr-lang-rust for Rust support
#
# Prerequisites (install manually):
#   - Python 3.8+, pip
#   - cmake, ninja-build
#   - aria2c (for parallel downloads)
#   - Rust toolchain (rustup)
#
# Usage:
#   ./scripts/zephyr/setup.sh [OPTIONS]
#
# Options:
#   --force            Overwrite existing workspace
#   --skip-sdk         Skip SDK installation (if already installed)
#
# Example:
#   ./scripts/zephyr/setup.sh
#   source ../nano-ros-workspace/env.sh
#   cd ../nano-ros-workspace
#   west build -b native_sim/native/64 nano-ros/examples/zephyr-talker

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NANO_ROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NANO_ROS_PARENT="$(dirname "$NANO_ROS_ROOT")"
NANO_ROS_NAME="$(basename "$NANO_ROS_ROOT")"
WORKSPACE_DIR="$NANO_ROS_PARENT/${NANO_ROS_NAME}-workspace"
DOWNLOAD_DIR="$SCRIPT_DIR/downloads"
SDK_INSTALL_DIR="$SCRIPT_DIR/sdk"

# Zephyr SDK configuration
ZEPHYR_SDK_VERSION="0.16.8"
ZEPHYR_SDK_TARBALL="zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64.tar.xz"
ZEPHYR_SDK_URL="https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/${ZEPHYR_SDK_TARBALL}"
ZEPHYR_SDK_SHA256="cb4e4012751e4526aaf1ec1e8ab9b4ded5681e2e01711b64f7a1b519ff7dbc6a"

# Parse arguments
FORCE=false
SKIP_SDK=false

while [[ $# -gt 0 ]]; do
    case $1 in
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
            echo "Create nano-ros Zephyr workspace at ../${NANO_ROS_NAME}-workspace/"
            echo ""
            echo "Options:"
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
log_info "SDK directory: $SDK_INSTALL_DIR"
log_info "Download cache: $DOWNLOAD_DIR"
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
check_command ninja || { log_warn "ninja not found"; MISSING=1; }
check_command aria2c || { log_warn "aria2c not found"; MISSING=1; }
check_command rustc || MISSING=1
check_command cargo || MISSING=1

if [ $MISSING -eq 1 ]; then
    echo ""
    log_error "Missing prerequisites. Please install:"
    echo "  sudo apt install python3 python3-pip python3-venv cmake ninja-build aria2 git"
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
# Download and Install Zephyr SDK
# =============================================================================

SDK_PATH="$SDK_INSTALL_DIR/zephyr-sdk-$ZEPHYR_SDK_VERSION"
SDK_TARBALL_PATH="$DOWNLOAD_DIR/$ZEPHYR_SDK_TARBALL"

download_sdk() {
    mkdir -p "$DOWNLOAD_DIR"

    log_info "Downloading Zephyr SDK $ZEPHYR_SDK_VERSION..."
    log_info "URL: $ZEPHYR_SDK_URL"

    # aria2c options:
    #   -x 16: 16 connections per server
    #   -s 16: split file into 16 parts
    #   -k 1M: minimum split size 1MB
    #   -c: continue partial downloads
    #   --checksum: verify sha256 after download
    aria2c \
        -x 16 \
        -s 16 \
        -k 1M \
        -c \
        --checksum=sha-256="$ZEPHYR_SDK_SHA256" \
        -d "$DOWNLOAD_DIR" \
        -o "$ZEPHYR_SDK_TARBALL" \
        "$ZEPHYR_SDK_URL"

    log_success "Download complete"
}

verify_sdk_checksum() {
    if [ ! -f "$SDK_TARBALL_PATH" ]; then
        return 1
    fi

    log_info "Verifying checksum..."
    local actual_sha256
    actual_sha256=$(sha256sum "$SDK_TARBALL_PATH" | cut -d' ' -f1)

    if [ "$actual_sha256" = "$ZEPHYR_SDK_SHA256" ]; then
        log_success "Checksum verified: $actual_sha256"
        return 0
    else
        log_warn "Checksum mismatch!"
        log_warn "  Expected: $ZEPHYR_SDK_SHA256"
        log_warn "  Actual:   $actual_sha256"
        return 1
    fi
}

install_sdk() {
    log_info "Extracting SDK to $SDK_INSTALL_DIR..."
    mkdir -p "$SDK_INSTALL_DIR"

    tar xf "$SDK_TARBALL_PATH" -C "$SDK_INSTALL_DIR"

    log_info "Running SDK setup..."
    cd "$SDK_PATH"
    ./setup.sh -t x86_64-zephyr-elf -t arm-zephyr-eabi -h -c

    log_success "Zephyr SDK installed"
}

if [ "$SKIP_SDK" = true ]; then
    log_info "Skipping SDK installation (--skip-sdk)"
elif [ -d "$SDK_PATH" ] && [ -f "$SDK_PATH/setup.sh" ]; then
    log_info "Zephyr SDK already installed at $SDK_PATH"
else
    # Check if tarball exists and has correct checksum
    if verify_sdk_checksum; then
        log_info "Using cached SDK tarball"
    else
        # Download (or resume) the SDK
        download_sdk
    fi

    # Install the SDK
    install_sdk
fi

export ZEPHYR_SDK_INSTALL_DIR="$SDK_PATH"

# =============================================================================
# Create Environment Script
# =============================================================================

create_env_script() {
    log_info "Creating environment script..."
    cat > "$WORKSPACE_DIR/env.sh" << ENVEOF
#!/bin/bash
# nano-ros Zephyr Environment
# Usage: source ../nano-ros-workspace/env.sh (from nano-ros dir)
#    or: source env.sh (from workspace dir)

WORKSPACE="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"

# Zephyr environment
source "\$WORKSPACE/zephyr/zephyr-env.sh"

# Zephyr SDK
export ZEPHYR_SDK_INSTALL_DIR="$SDK_PATH"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr

# nano-ros paths
export NANO_ROS_ROOT="\$WORKSPACE/$NANO_ROS_NAME"

# Local bin
export PATH="\$HOME/.local/bin:\$PATH"

echo "nano-ros Zephyr environment ready"
echo "  ZEPHYR_BASE: \$ZEPHYR_BASE"
echo "  ZEPHYR_SDK: $SDK_PATH"
echo "  NANO_ROS_ROOT: \$NANO_ROS_ROOT"
echo ""
echo "Build example:"
echo "  cd \$WORKSPACE"
echo "  west build -b native_sim/native/64 $NANO_ROS_NAME/examples/zephyr-talker"
ENVEOF
    chmod +x "$WORKSPACE_DIR/env.sh"
}

# =============================================================================
# Initialize Workspace
# =============================================================================

if [ -d "$WORKSPACE_DIR/.west" ]; then
    if [ "$FORCE" = true ]; then
        log_warn "Removing existing workspace..."
        rm -rf "$WORKSPACE_DIR"
    else
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
        echo "  cd $WORKSPACE_DIR"
        echo "  west build -b native_sim/native/64 $NANO_ROS_NAME/examples/zephyr-talker"
        exit 0
    fi
fi

log_info "Initializing workspace..."
mkdir -p "$WORKSPACE_DIR"
cd "$WORKSPACE_DIR"

# Create manifest directory with west.yml, then replace with symlink
# (west init -l follows symlinks during init, so we copy first)
mkdir -p "$WORKSPACE_DIR/$NANO_ROS_NAME"
cp "$NANO_ROS_ROOT/west.yml" "$WORKSPACE_DIR/$NANO_ROS_NAME/west.yml"

# Initialize west
west init -l "$WORKSPACE_DIR/$NANO_ROS_NAME"

# Replace with symlink to real nano-ros
rm -rf "$WORKSPACE_DIR/$NANO_ROS_NAME"
ln -sf "$NANO_ROS_ROOT" "$WORKSPACE_DIR/$NANO_ROS_NAME"

log_info "Fetching Zephyr and modules (this may take a while)..."
west update

# Install Zephyr Python dependencies
log_info "Installing Zephyr Python dependencies..."
pip3 install --user -r "$WORKSPACE_DIR/zephyr/scripts/requirements.txt"

# Create environment script
create_env_script

# Create symlink from nano-ros to workspace (for scripts to find it)
WORKSPACE_SYMLINK="$NANO_ROS_ROOT/zephyr-workspace"
log_info "Creating workspace symlink: $WORKSPACE_SYMLINK -> $WORKSPACE_DIR"
ln -sfn "$WORKSPACE_DIR" "$WORKSPACE_SYMLINK"

# =============================================================================
# Summary
# =============================================================================

echo ""
log_success "========================================"
log_success "  Workspace setup complete!"
log_success "========================================"
echo ""
echo "Workspace: $WORKSPACE_DIR"
echo "Symlink:   $WORKSPACE_SYMLINK"
echo "SDK:       $SDK_PATH"
echo ""
echo "Structure:"
echo "  $NANO_ROS_NAME/  -> $NANO_ROS_ROOT (symlink)"
echo "  zephyr/          - Zephyr RTOS v3.7.0"
echo "  modules/         - Zephyr modules (lang/rust, zenoh-pico, HALs)"
echo ""
echo "Next steps:"
echo ""
echo "  1. Source the environment:"
echo "     source $WORKSPACE_DIR/env.sh"
echo ""
echo "  2. Build an example:"
echo "     cd $WORKSPACE_DIR"
echo "     west build -b native_sim/native/64 $NANO_ROS_NAME/examples/zephyr-talker"
echo ""
echo "  3. Run:"
echo "     ./build/zephyr/zephyr.exe"
echo ""
echo "  4. For networking tests, configure TAP interface:"
echo "     sudo $NANO_ROS_ROOT/scripts/zephyr/setup-network.sh"
echo ""
