#!/bin/bash
# Check prerequisites for nano-ros tests
#
# Source this file after utils.sh:
#   source "$(dirname "$0")/../common/utils.sh"
#   source "$(dirname "$0")/../common/prerequisites.sh"

# Check if zenohd is available
check_zenohd() {
    if command -v zenohd &> /dev/null; then
        log_success "zenohd found: $(which zenohd)"
        return 0
    else
        log_error "zenohd not found in PATH"
        echo "  Install zenoh or add it to PATH"
        return 1
    fi
}

# Check if ROS 2 is available
check_ros2() {
    local ros_distro="${1:-humble}"
    local ros_setup="/opt/ros/$ros_distro/setup.bash"

    if [ -f "$ros_setup" ]; then
        log_success "ROS 2 $ros_distro found"
        return 0
    else
        log_error "ROS 2 $ros_distro not found at $ros_setup"
        return 1
    fi
}

# Check if rmw_zenoh_cpp is installed
check_rmw_zenoh() {
    local ros_distro="${1:-humble}"

    # Source ROS 2 first
    # shellcheck source=/dev/null
    source "/opt/ros/$ros_distro/setup.bash" 2>/dev/null || return 1

    if ros2 pkg list 2>/dev/null | grep -q rmw_zenoh_cpp; then
        log_success "rmw_zenoh_cpp found"
        return 0
    else
        log_error "rmw_zenoh_cpp not installed"
        echo "  Install with: sudo apt install ros-$ros_distro-rmw-zenoh-cpp"
        return 1
    fi
}

# Check if nano-ros binaries are built
check_nano_ros_built() {
    local missing=0

    if [ -x "$TALKER_BIN" ]; then
        log_success "nano-ros talker built"
    else
        log_warn "nano-ros talker not built"
        missing=1
    fi

    if [ -x "$LISTENER_BIN" ]; then
        log_success "nano-ros listener built"
    else
        log_warn "nano-ros listener not built"
        missing=1
    fi

    return $missing
}

# Check all prerequisites for nano-to-nano tests
check_nano2nano_prerequisites() {
    log_header "Checking nano2nano Prerequisites"

    local failed=0
    check_zenohd || failed=1
    check_nano_ros_built || {
        log_info "Building nano-ros..."
        build_nano_ros || failed=1
    }

    return $failed
}

# Check all prerequisites for RMW interop tests
check_rmw_prerequisites() {
    local ros_distro="${1:-humble}"

    log_header "Checking RMW Interop Prerequisites"

    local failed=0
    check_zenohd || failed=1
    check_ros2 "$ros_distro" || failed=1
    check_rmw_zenoh "$ros_distro" || failed=1
    check_nano_ros_built || {
        log_info "Building nano-ros..."
        build_nano_ros || failed=1
    }

    return $failed
}

# Check if z_sub/z_pub are available (for debugging)
check_zenoh_examples() {
    if [ -x "$Z_SUB" ]; then
        log_success "z_sub found"
    else
        log_warn "z_sub not found at $Z_SUB (optional, for debugging)"
    fi

    if [ -x "$Z_PUB" ]; then
        log_success "z_pub found"
    else
        log_warn "z_pub not found at $Z_PUB (optional, for debugging)"
    fi
}
