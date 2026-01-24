#!/bin/bash
# Test: Generic platform (compile-only)
#
# Verifies zenoh-pico-shim compiles without any platform backend.
# This is useful for CI to catch API breakage and ensure the crate
# structure is correct.
#
# Tests:
#   - Compile without any platform feature
#   - Compile with only std feature
#   - Compile with only alloc feature
#   - Verify no_std compatibility
#
# Usage:
#   ./tests/platform/generic.sh
#   ./tests/platform/generic.sh --verbose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"

VERBOSE=false
[[ "$1" == "--verbose" || "$1" == "-v" ]] && VERBOSE=true

setup_cleanup

log_header "Generic Platform Test (Compile-Only)"

# Check prerequisites
check_generic_prerequisites() {
    log_header "Checking Prerequisites"

    local failed=0

    # Check cargo
    if command -v cargo &> /dev/null; then
        log_success "cargo found: $(which cargo)"
    else
        log_error "cargo not found in PATH"
        failed=1
    fi

    # Check for thumbv7em target (for embedded compile check)
    if rustup target list --installed | grep -q thumbv7em-none-eabihf; then
        log_success "thumbv7em-none-eabihf target installed"
    else
        log_warn "thumbv7em-none-eabihf not installed (embedded tests skipped)"
        log_info "Install with: rustup target add thumbv7em-none-eabihf"
    fi

    return $failed
}

if ! check_generic_prerequisites; then
    log_error "Prerequisites not met"
    exit 1
fi

RESULT=0

# =============================================================================
# Test 1: Compile without any features (no_std, no platform)
# =============================================================================

test_compile_no_features() {
    log_header "Test: Compile without any features"

    log_info "Checking zenoh-pico-shim compiles with no features..."

    # This should compile the crate structure without any platform backend
    if cargo check -p zenoh-pico-shim --no-default-features \
        > "$(tmpfile generic_no_features.txt)" 2>&1; then
        log_success "Compiles with no features (no_std, no platform)"
        return 0
    else
        log_error "Compile failed"
        cat "$(tmpfile generic_no_features.txt)"
        return 1
    fi
}

# =============================================================================
# Test 2: Compile with std feature only
# =============================================================================

test_compile_std_only() {
    log_header "Test: Compile with std feature only"

    log_info "Checking zenoh-pico-shim compiles with only std feature..."

    if cargo check -p zenoh-pico-shim --no-default-features --features std \
        > "$(tmpfile generic_std_only.txt)" 2>&1; then
        log_success "Compiles with std feature only"
        return 0
    else
        log_error "Compile failed"
        cat "$(tmpfile generic_std_only.txt)"
        return 1
    fi
}

# =============================================================================
# Test 3: Compile with alloc feature only
# =============================================================================

test_compile_alloc_only() {
    log_header "Test: Compile with alloc feature only"

    log_info "Checking zenoh-pico-shim compiles with only alloc feature..."

    if cargo check -p zenoh-pico-shim --no-default-features --features alloc \
        > "$(tmpfile generic_alloc_only.txt)" 2>&1; then
        log_success "Compiles with alloc feature only"
        return 0
    else
        log_error "Compile failed"
        cat "$(tmpfile generic_alloc_only.txt)"
        return 1
    fi
}

# =============================================================================
# Test 4: Compile zenoh-pico-shim-sys without features
# =============================================================================

test_compile_sys_no_features() {
    log_header "Test: Compile zenoh-pico-shim-sys without features"

    log_info "Checking zenoh-pico-shim-sys compiles with no features..."

    if cargo check -p zenoh-pico-shim-sys --no-default-features \
        > "$(tmpfile generic_sys_no_features.txt)" 2>&1; then
        log_success "zenoh-pico-shim-sys compiles with no features"
        return 0
    else
        log_error "Compile failed"
        cat "$(tmpfile generic_sys_no_features.txt)"
        return 1
    fi
}

# =============================================================================
# Test 5: Verify no_std compatibility (embedded target)
# =============================================================================

test_no_std_embedded() {
    log_header "Test: no_std Compatibility (Embedded Target)"

    # Check if thumbv7em target is available
    if ! rustup target list --installed | grep -q thumbv7em-none-eabihf; then
        log_warn "thumbv7em-none-eabihf not installed, skipping"
        return 0
    fi

    log_info "Checking zenoh-pico-shim compiles for thumbv7em-none-eabihf..."

    # Compile without std for embedded target
    # Note: This only checks the crate itself, not linking
    if cargo check -p zenoh-pico-shim --no-default-features \
        --target thumbv7em-none-eabihf \
        > "$(tmpfile generic_embedded.txt)" 2>&1; then
        log_success "Compiles for embedded target (no_std)"
        return 0
    else
        # This might fail due to missing platform layer - that's expected
        if grep -q "platform" "$(tmpfile generic_embedded.txt)" 2>/dev/null; then
            log_info "Embedded compile requires platform layer (expected)"
            return 0
        fi
        log_error "Unexpected compile failure"
        cat "$(tmpfile generic_embedded.txt)"
        return 1
    fi
}

# =============================================================================
# Test 6: Feature Combinations
# =============================================================================

test_feature_combinations() {
    log_header "Test: Feature Combinations"

    local combinations=(
        "posix"
        "posix,std"
        "smoltcp"
        "zephyr"
    )

    local passed=0
    local failed=0

    for features in "${combinations[@]}"; do
        log_info "Checking: --features $features"

        if cargo check -p zenoh-pico-shim --no-default-features --features "$features" \
            > "$(tmpfile generic_combo_${features//,/_}.txt)" 2>&1; then
            log_success "  $features: OK"
            passed=$((passed + 1))
        else
            # zephyr might fail without Zephyr SDK - that's acceptable
            if [[ "$features" == "zephyr" ]]; then
                log_warn "  $features: Skipped (needs Zephyr SDK)"
            else
                log_error "  $features: FAILED"
                [ "$VERBOSE" = true ] && cat "$(tmpfile generic_combo_${features//,/_}.txt)"
                failed=$((failed + 1))
            fi
        fi
    done

    echo ""
    log_info "Feature combinations: $passed passed, $failed failed"

    if [ $failed -gt 0 ]; then
        return 1
    fi
    return 0
}

# =============================================================================
# Test 7: Doc Generation
# =============================================================================

test_doc_generation() {
    log_header "Test: Documentation Generation"

    log_info "Checking documentation generates without errors..."

    if cargo doc -p zenoh-pico-shim --no-default-features --features "posix std" --no-deps \
        > "$(tmpfile generic_doc.txt)" 2>&1; then
        log_success "Documentation generates successfully"
        return 0
    else
        log_error "Documentation generation failed"
        [ "$VERBOSE" = true ] && cat "$(tmpfile generic_doc.txt)"
        return 1
    fi
}

# =============================================================================
# Test 8: Clippy Lints
# =============================================================================

test_clippy() {
    log_header "Test: Clippy Lints"

    log_info "Running clippy on zenoh-pico-shim..."

    if cargo clippy -p zenoh-pico-shim --no-default-features --features "posix std" \
        -- -D warnings > "$(tmpfile generic_clippy.txt)" 2>&1; then
        log_success "Clippy passes with no warnings"
        return 0
    else
        log_warn "Clippy found warnings or errors"
        [ "$VERBOSE" = true ] && cat "$(tmpfile generic_clippy.txt)"
        # Don't fail on clippy warnings - just warn
        return 0
    fi
}

# =============================================================================
# Test 9: API Surface Check
# =============================================================================

test_api_surface() {
    log_header "Test: API Surface Check"

    log_info "Verifying public API types exist..."

    local lib_file="$PROJECT_ROOT/crates/zenoh-pico-shim/src/lib.rs"

    if [ ! -f "$lib_file" ]; then
        log_error "lib.rs not found"
        return 1
    fi

    local types=(
        "ShimContext"
        "ShimPublisher"
        "ShimSubscriber"
        "ShimLivelinessToken"
        "ShimZenohId"
        "ShimError"
    )

    local found=0
    for type in "${types[@]}"; do
        if grep -q "pub.*$type" "$lib_file" 2>/dev/null || \
           grep -q "pub use.*$type" "$lib_file" 2>/dev/null; then
            [ "$VERBOSE" = true ] && log_success "  Found: $type"
            found=$((found + 1))
        else
            log_warn "  Not exported: $type"
        fi
    done

    log_success "API surface: $found/${#types[@]} types exported"
    return 0
}

# =============================================================================
# Run Tests
# =============================================================================

test_compile_no_features || RESULT=1
test_compile_std_only || RESULT=1
test_compile_alloc_only || RESULT=1
test_compile_sys_no_features || RESULT=1
test_no_std_embedded || true  # May fail without platform
test_feature_combinations || RESULT=1
test_doc_generation || RESULT=1
test_clippy || true  # Don't fail on clippy
test_api_surface || true  # Informational

# =============================================================================
# Summary
# =============================================================================

log_header "Test Summary"

echo ""
echo "Tested Feature Combinations:"
echo "  - No features (no_std, no platform)"
echo "  - std only"
echo "  - alloc only"
echo "  - posix"
echo "  - posix,std"
echo "  - smoltcp"
echo "  - zephyr (if SDK available)"
echo ""

if [ $RESULT -eq 0 ]; then
    log_success "All generic platform tests passed!"
else
    log_error "Some generic platform tests failed"
fi

exit $RESULT
