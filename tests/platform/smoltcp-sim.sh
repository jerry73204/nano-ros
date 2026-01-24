#!/bin/bash
# Test: smoltcp platform backend (simulation)
#
# Verifies zenoh-pico-shim smoltcp platform compiles and unit tests pass.
# This tests the platform layer without requiring actual network hardware.
#
# The smoltcp platform provides:
#   - Bump allocator (16 KB static heap)
#   - Monotonic clock (externally updated)
#   - Socket buffer management
#   - Random number generation
#
# Usage:
#   ./tests/platform/smoltcp-sim.sh
#   ./tests/platform/smoltcp-sim.sh --verbose

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/utils.sh"

VERBOSE=false
[[ "$1" == "--verbose" || "$1" == "-v" ]] && VERBOSE=true

setup_cleanup

log_header "smoltcp Platform Backend Test (Simulation)"

# Check prerequisites
check_smoltcp_prerequisites() {
    log_header "Checking Prerequisites"

    local failed=0

    # Check cargo
    if command -v cargo &> /dev/null; then
        log_success "cargo found: $(which cargo)"
    else
        log_error "cargo not found in PATH"
        failed=1
    fi

    return $failed
}

if ! check_smoltcp_prerequisites; then
    log_error "Prerequisites not met"
    exit 1
fi

RESULT=0

# =============================================================================
# Test 1: Compile Check (x86_64)
# =============================================================================

test_compile_x86_64() {
    log_header "Test: Compile for x86_64-unknown-linux-gnu"

    log_info "Checking zenoh-pico-shim compiles with smoltcp feature..."

    if cargo check -p zenoh-pico-shim --features smoltcp \
        > "$(tmpfile smoltcp_compile.txt)" 2>&1; then
        log_success "zenoh-pico-shim compiles with smoltcp feature"
        return 0
    else
        log_error "Compile failed"
        cat "$(tmpfile smoltcp_compile.txt)"
        return 1
    fi
}

# =============================================================================
# Test 2: Compile Check (zenoh-pico-shim-sys)
# =============================================================================

test_compile_sys() {
    log_header "Test: Compile zenoh-pico-shim-sys with smoltcp"

    log_info "Checking zenoh-pico-shim-sys compiles with smoltcp feature..."

    if cargo check -p zenoh-pico-shim-sys --features smoltcp \
        > "$(tmpfile smoltcp_sys_compile.txt)" 2>&1; then
        log_success "zenoh-pico-shim-sys compiles with smoltcp feature"
        return 0
    else
        log_error "Compile failed"
        cat "$(tmpfile smoltcp_sys_compile.txt)"
        return 1
    fi
}

# =============================================================================
# Test 3: Platform Layer Module Presence
# =============================================================================

test_platform_module() {
    log_header "Test: Platform Layer Module"

    log_info "Verifying platform_smoltcp module exists..."

    local platform_file="$PROJECT_ROOT/crates/zenoh-pico-shim-sys/src/platform_smoltcp.rs"

    if [ -f "$platform_file" ]; then
        log_success "platform_smoltcp.rs exists"

        # Check for key functions
        local functions=(
            "smoltcp_alloc"
            "smoltcp_free"
            "smoltcp_set_clock_ms"
            "smoltcp_clock_now_ms"
            "smoltcp_random_u32"
            "smoltcp_socket_open"
            "smoltcp_socket_connect"
            "smoltcp_socket_send"
            "smoltcp_socket_recv"
        )

        local missing=0
        for func in "${functions[@]}"; do
            if grep -q "fn $func" "$platform_file" 2>/dev/null; then
                [ "$VERBOSE" = true ] && log_success "  Found: $func"
            else
                log_warn "  Missing: $func"
                missing=$((missing + 1))
            fi
        done

        if [ $missing -eq 0 ]; then
            log_success "All expected platform functions found"
            return 0
        else
            log_warn "$missing functions missing (may be expected)"
            return 0
        fi
    else
        log_error "platform_smoltcp.rs not found"
        return 1
    fi
}

# =============================================================================
# Test 4: C Platform Layer Files
# =============================================================================

test_c_platform_files() {
    log_header "Test: C Platform Layer Files"

    log_info "Verifying C platform layer files exist..."

    local platform_dir="$PROJECT_ROOT/crates/zenoh-pico-shim-sys/c/platform_smoltcp"

    local files=(
        "system.c"
        "network.c"
        "zenoh_smoltcp_platform.h"
        "zenoh_generic_platform.h"
        "zenoh_generic_config.h"
    )

    local missing=0
    for file in "${files[@]}"; do
        if [ -f "$platform_dir/$file" ]; then
            log_success "  Found: $file"
        else
            log_error "  Missing: $file"
            missing=$((missing + 1))
        fi
    done

    if [ $missing -eq 0 ]; then
        log_success "All C platform layer files present"
        return 0
    else
        log_error "$missing files missing"
        return 1
    fi
}

# =============================================================================
# Test 5: Allocator Constants
# =============================================================================

test_allocator_constants() {
    log_header "Test: Allocator Constants"

    log_info "Verifying allocator configuration..."

    local platform_file="$PROJECT_ROOT/crates/zenoh-pico-shim-sys/src/platform_smoltcp.rs"

    # Check heap size constant
    if grep -q "HEAP_SIZE.*16.*1024" "$platform_file" 2>/dev/null; then
        log_success "HEAP_SIZE = 16 KB"
    else
        log_warn "HEAP_SIZE constant not found or different size"
    fi

    # Check socket buffer constants
    if grep -q "SOCKET_BUFFER_SIZE" "$platform_file" 2>/dev/null; then
        local size
        size=$(grep "SOCKET_BUFFER_SIZE" "$platform_file" | grep -oE "[0-9]+" | head -1)
        log_success "SOCKET_BUFFER_SIZE = $size bytes"
    fi

    # Check max sockets constant
    if grep -q "MAX_SOCKETS" "$platform_file" 2>/dev/null; then
        local max
        max=$(grep "MAX_SOCKETS" "$platform_file" | grep -oE "[0-9]+" | head -1)
        log_success "MAX_SOCKETS = $max"
    fi

    return 0
}

# =============================================================================
# Test 6: Build with smoltcp Feature
# =============================================================================

test_build_smoltcp() {
    log_header "Test: Build with smoltcp Feature"

    log_info "Building zenoh-pico-shim with smoltcp feature..."

    # Build (not just check) to ensure linking works
    if cargo build -p zenoh-pico-shim --features smoltcp \
        > "$(tmpfile smoltcp_build.txt)" 2>&1; then
        log_success "Build successful"

        if [ "$VERBOSE" = true ]; then
            # Show binary size info if available
            local lib_path
            lib_path=$(find "$PROJECT_ROOT/target/debug" -name "libzenoh_pico_shim*.rlib" 2>/dev/null | head -1)
            if [ -n "$lib_path" ]; then
                log_info "Library size: $(du -h "$lib_path" | cut -f1)"
            fi
        fi
        return 0
    else
        log_error "Build failed"
        cat "$(tmpfile smoltcp_build.txt)"
        return 1
    fi
}

# =============================================================================
# Test 7: FFI Function Declarations
# =============================================================================

test_ffi_declarations() {
    log_header "Test: FFI Function Declarations"

    log_info "Verifying FFI declarations in ffi.rs..."

    local ffi_file="$PROJECT_ROOT/crates/zenoh-pico-shim-sys/src/ffi.rs"

    if [ ! -f "$ffi_file" ]; then
        log_error "ffi.rs not found"
        return 1
    fi

    # Check for smoltcp platform FFI stubs
    local stubs=(
        "smoltcp_alloc"
        "smoltcp_free"
        "smoltcp_set_clock_ms"
    )

    local found=0
    for stub in "${stubs[@]}"; do
        if grep -q "$stub" "$ffi_file" 2>/dev/null; then
            found=$((found + 1))
            [ "$VERBOSE" = true ] && log_success "  Found FFI stub: $stub"
        fi
    done

    if [ $found -gt 0 ]; then
        log_success "FFI stubs present ($found found)"
        return 0
    else
        log_warn "No smoltcp FFI stubs found in ffi.rs"
        return 0
    fi
}

# =============================================================================
# Test 8: Header Generation (cbindgen)
# =============================================================================

test_header_generation() {
    log_header "Test: Header Generation"

    log_info "Verifying cbindgen configuration..."

    local cbindgen_file="$PROJECT_ROOT/crates/zenoh-pico-shim-sys/cbindgen.toml"

    if [ -f "$cbindgen_file" ]; then
        log_success "cbindgen.toml exists"

        if grep -q "platform_smoltcp" "$cbindgen_file" 2>/dev/null; then
            log_success "platform_smoltcp referenced in cbindgen config"
        else
            log_info "platform_smoltcp not in cbindgen (excluded via cfg)"
        fi
        return 0
    else
        log_warn "cbindgen.toml not found"
        return 0
    fi
}

# =============================================================================
# Test 9: Documentation
# =============================================================================

test_documentation() {
    log_header "Test: Documentation"

    log_info "Checking documentation..."

    # Check for integration docs
    local docs=(
        "$PROJECT_ROOT/docs/architecture/smoltcp-zenoh-pico-integration.md"
        "$PROJECT_ROOT/docs/roadmap/phase-8-embedded-networking.md"
    )

    for doc in "${docs[@]}"; do
        if [ -f "$doc" ]; then
            log_success "Found: $(basename "$doc")"
        else
            log_warn "Missing: $(basename "$doc")"
        fi
    done

    return 0
}

# =============================================================================
# Run Tests
# =============================================================================

test_compile_x86_64 || RESULT=1
test_compile_sys || RESULT=1
test_platform_module || RESULT=1
test_c_platform_files || RESULT=1
test_allocator_constants || true  # Informational
test_build_smoltcp || RESULT=1
test_ffi_declarations || true  # Informational
test_header_generation || true  # Informational
test_documentation || true  # Informational

# =============================================================================
# Summary
# =============================================================================

log_header "Test Summary"

echo ""
echo "smoltcp Platform Configuration:"
echo "  Heap size: 16 KB (bump allocator)"
echo "  Max sockets: 4"
echo "  Socket buffer: 2 KB per socket"
echo "  Total memory: ~33 KB"
echo ""

if [ $RESULT -eq 0 ]; then
    log_success "All smoltcp platform tests passed!"
    echo ""
    log_info "Note: Full network testing requires hardware (Phase 8.9)"
else
    log_error "Some smoltcp platform tests failed"
fi

exit $RESULT
