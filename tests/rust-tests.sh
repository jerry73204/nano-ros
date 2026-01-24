#!/usr/bin/env bash
# Rust-based integration tests wrapper
# Invokes tests from crates/nano-ros-tests

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$SCRIPT_DIR/common/utils.sh"

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS] [TEST_NAME]

Run Rust-based integration tests from crates/nano-ros-tests.

TEST NAMES:
  emulator    Run QEMU emulator tests
  nano2nano   Run native pub/sub tests
  platform    Run platform detection tests
  all         Run all Rust tests (default)

OPTIONS:
  -v, --verbose    Show verbose output
  -h, --help       Show this help

EXAMPLES:
  $(basename "$0")                    # Run all tests
  $(basename "$0") emulator           # Run only emulator tests
  $(basename "$0") nano2nano          # Run native pub/sub tests
  $(basename "$0") platform           # Run platform tests
  $(basename "$0") -v all             # Verbose all tests
EOF
}

VERBOSE=""
TEST_NAME="all"

while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--verbose)
            VERBOSE="--verbose"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        emulator|nano2nano|platform|all)
            TEST_NAME="$1"
            shift
            ;;
        *)
            log_error "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

log_section "Rust Integration Tests"

cd "$PROJECT_ROOT"

run_test() {
    local test_name="$1"
    log_info "Running $test_name tests..."

    if cargo test -p nano-ros-tests --test "$test_name" $VERBOSE -- --nocapture 2>&1; then
        log_success "$test_name tests passed"
        return 0
    else
        log_error "$test_name tests failed"
        return 1
    fi
}

FAILED=0

case "$TEST_NAME" in
    emulator)
        run_test emulator || FAILED=1
        ;;
    nano2nano)
        run_test nano2nano || FAILED=1
        ;;
    platform)
        run_test platform || FAILED=1
        ;;
    all)
        log_info "Running all Rust integration tests..."

        run_test emulator || FAILED=1
        run_test nano2nano || FAILED=1
        run_test platform || FAILED=1
        ;;
esac

if [[ $FAILED -eq 0 ]]; then
    log_success "All requested Rust tests passed"
else
    log_error "Some Rust tests failed"
fi

exit $FAILED
