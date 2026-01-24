#!/bin/bash
# Common utilities for nano-ros tests
#
# Source this file in test scripts:
#   source "$(dirname "$0")/../common/utils.sh"

# Get project root directory
TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ROOT="$(dirname "$TESTS_DIR")"

# Paths - native examples are standalone packages with their own target directories
TALKER_BIN="$PROJECT_ROOT/examples/native-talker/target/release/talker"
LISTENER_BIN="$PROJECT_ROOT/examples/native-listener/target/release/listener"
Z_SUB="$PROJECT_ROOT/crates/zenoh-pico-shim-sys/zenoh-pico/build/examples/z_sub"
Z_PUB="$PROJECT_ROOT/crates/zenoh-pico-shim-sys/zenoh-pico/build/examples/z_pub"

# Default configuration
ZENOH_LOCATOR="${ZENOH_LOCATOR:-tcp/127.0.0.1:7447}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
TEST_TIMEOUT="${TEST_TIMEOUT:-15}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Logging functions
log_info()    { echo -e "${BLUE}[INFO]${NC} $*"; }
log_success() { echo -e "${GREEN}[PASS]${NC} $*"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error()   { echo -e "${RED}[FAIL]${NC} $*"; }
log_header()  { echo -e "\n${CYAN}=== $* ===${NC}"; }

# PIDs for cleanup
declare -a CLEANUP_PIDS=()
ZENOHD_PID=""

# Register a PID for cleanup
register_pid() {
    CLEANUP_PIDS+=("$1")
}

# Cleanup function - kills all registered processes
cleanup() {
    log_info "Cleaning up..."

    # Kill registered PIDs
    for pid in "${CLEANUP_PIDS[@]}"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done

    # Kill zenohd
    if [ -n "$ZENOHD_PID" ] && kill -0 "$ZENOHD_PID" 2>/dev/null; then
        kill "$ZENOHD_PID" 2>/dev/null || true
    fi

    # Also kill by name as fallback
    pkill -x zenohd 2>/dev/null || true
    pkill -f "/talker" 2>/dev/null || true
    pkill -f "/listener" 2>/dev/null || true

    # Clean up temp files
    rm -f /tmp/zenohd_test.log /tmp/nano_*.txt /tmp/ros2_*.txt

    CLEANUP_PIDS=()
    ZENOHD_PID=""
}

# Setup trap for cleanup
setup_cleanup() {
    trap cleanup EXIT INT TERM
}

# Start zenoh router
start_zenohd() {
    log_info "Starting zenohd at $ZENOH_LOCATOR..."

    # Kill any existing zenohd
    pkill -x zenohd 2>/dev/null || true
    sleep 1

    zenohd --listen "$ZENOH_LOCATOR" > /tmp/zenohd_test.log 2>&1 &
    ZENOHD_PID=$!
    sleep 2

    if ! kill -0 "$ZENOHD_PID" 2>/dev/null; then
        log_error "Failed to start zenohd"
        cat /tmp/zenohd_test.log
        return 1
    fi

    log_success "zenohd started (PID: $ZENOHD_PID)"
    return 0
}

# Build nano-ros examples
build_nano_ros() {
    log_info "Building nano-ros examples..."

    # Build native-talker (standalone package) with zenoh transport
    if ! (cd "$PROJECT_ROOT/examples/native-talker" && cargo build --release --features zenoh 2>&1 | tail -5); then
        log_error "Failed to build native-talker"
        return 1
    fi

    # Build native-listener (standalone package) with zenoh transport
    if ! (cd "$PROJECT_ROOT/examples/native-listener" && cargo build --release --features zenoh 2>&1 | tail -5); then
        log_error "Failed to build native-listener"
        return 1
    fi

    log_success "Build complete"
    return 0
}

# Source ROS 2 environment
setup_ros2_env() {
    local ros_distro="${1:-humble}"
    local ros_setup="/opt/ros/$ros_distro/setup.bash"

    if [ ! -f "$ros_setup" ]; then
        log_error "ROS 2 $ros_distro not found at $ros_setup"
        return 1
    fi

    # shellcheck source=/dev/null
    source "$ros_setup"

    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
    export ZENOH_ROUTER_CHECK_ATTEMPTS=0
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"$ZENOH_LOCATOR\"]"

    log_info "ROS 2 $ros_distro environment configured"
    return 0
}

# Wait for a pattern in a file with timeout
wait_for_pattern() {
    local file="$1"
    local pattern="$2"
    local timeout="${3:-$TEST_TIMEOUT}"
    local interval="${4:-0.5}"

    local elapsed=0
    while [ "$elapsed" -lt "$timeout" ]; do
        if grep -q "$pattern" "$file" 2>/dev/null; then
            return 0
        fi
        sleep "$interval"
        elapsed=$((elapsed + 1))
    done
    return 1
}

# Count pattern occurrences in file
count_pattern() {
    local file="$1"
    local pattern="$2"
    grep -c "$pattern" "$file" 2>/dev/null || echo 0
}

# Run a command with timeout and capture output
run_with_timeout() {
    local timeout="$1"
    local output_file="$2"
    shift 2

    timeout "$timeout" "$@" > "$output_file" 2>&1 &
    local pid=$!
    register_pid "$pid"
    echo "$pid"
}

# Print test result summary
print_result() {
    local test_name="$1"
    local result="$2"

    if [ "$result" -eq 0 ]; then
        log_success "$test_name"
    else
        log_error "$test_name"
    fi
}
