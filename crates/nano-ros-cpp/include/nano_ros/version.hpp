#pragma once

/// @file version.hpp
/// @brief Version information for nano-ros

#include <cstdint>
#include <string>

namespace nano_ros {

/// @brief Version information structure
struct Version {
    uint32_t major;
    uint32_t minor;
    uint32_t patch;

    /// @brief Get version as string (e.g., "0.1.0")
    std::string to_string() const {
        return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
    }
};

/// @brief Get the nano-ros library version
/// @return Version struct with major, minor, patch
Version get_version();

/// @brief Check if zenoh transport support is available
/// @return true if zenoh support is compiled in
bool has_zenoh_support();

}  // namespace nano_ros
