# cargo-ros2 Integration Test Workspace

Complex colcon workspace demonstrating cargo-ros2 with **standard ROS messages and custom interfaces**.

## Prerequisites

1. Install cargo-ros2 and colcon-ros-cargo:
   ```bash
   cd ..
   just install  # Installs both cargo-ros2 and colcon-ros-cargo
   ```

2. Source ROS 2:
   ```bash
   source /opt/ros/jazzy/setup.bash  # or your ROS distro
   ```

3. Install test dependencies:
   ```bash
   # Option 1: Use rosdep (recommended)
   rosdep install --from-paths src --ignore-src -r -y

   # Option 2: Manual installation
   sudo apt install -y \
     ros-humble-test-msgs \
     ros-humble-moveit-msgs \
     ros-humble-control-msgs \
     ros-humble-nav2-msgs \
     ros-humble-tf2-msgs \
     ros-humble-trajectory-msgs \
     ros-humble-diagnostic-msgs \
     ros-humble-composition-interfaces \
     ros-humble-rosbag2-interfaces
   ```

## Packages

- **robot_interfaces**: Custom messages, services, and actions (ament_cmake)
- **robot_controller**: Rust node using standard + custom interfaces (ament_cargo)

## Building

### One-Step Build (Recommended)
```bash
just build
# or
colcon build --symlink-install
```

### Step-by-Step Build
```bash
# 1. Build custom interfaces first
just build-interfaces

# 2. Build Rust node (depends on interfaces)
just build-rust
```

## What This Tests

### Standard ROS Types
✓ std_msgs (String, Header, Bool, Int32, Float64, etc.)
✓ geometry_msgs (Point, Pose, Twist, Transform, etc.)
✓ sensor_msgs (Imu, LaserScan, Image, etc.)

### Custom Interface Types
✓ Custom messages (RobotStatus, SensorReading)
✓ Custom services (SetMode)
✓ Custom actions (Navigate)

### Enhanced Test Coverage (NEW)
✓ **Parser edge cases** (test_msgs): Bounded sequences/strings, wide strings, constants, defaults, deep nesting
✓ **Motion planning** (moveit_msgs): Complex nested trajectories, collision objects, planning scenes
✓ **Robot control** (control_msgs): Controller states, PID values, multi-DOF states, gripper control
✓ **Navigation** (nav2_msgs): 2D/3D grids, costmaps, particle filters, behavior trees
✓ **Trajectories** (trajectory_msgs): Joint trajectories, multi-DOF trajectories, waypoints
✓ **Diagnostics** (diagnostic_msgs): System health, key-value pairs, diagnostic arrays
✓ **Transforms** (tf2_msgs): Transform hierarchies, time-stamped data
✓ **Advanced services**: Multi-package dependencies, complex request/response
✓ **Advanced actions**: Complex goal/result/feedback patterns with real-time updates

### Integration Features
✓ cargo-ros2 automatic binding generation
✓ colcon-ros-cargo integration
✓ Multi-package dependency resolution
✓ Ament installation layout
✓ Caching and incremental builds

---

**📖 For detailed test coverage documentation, see TEST_COVERAGE.md**

## Expected Workflow

1. **colcon build** → Builds robot_interfaces (C++ rosidl)
2. **colcon-ros-cargo** detects robot_controller (ament_cargo)
3. **cargo-ros2** discovers dependencies:
   - std_msgs, geometry_msgs, sensor_msgs (system packages)
   - robot_interfaces (workspace package)
4. **cargo-ros2** generates Rust bindings for all packages
5. **cargo-ros2** caches generated bindings
6. **cargo** builds robot_controller
7. **cargo-ros2** installs to ament layout

## Running

```bash
# Source workspace
source install/setup.bash

# Run the node
just run
# or
ros2 run robot_controller robot_controller
```

**Expected Output:**
```
=== Robot Controller Node ===

--- Standard ROS Messages ---
std_msgs::String: ...
std_msgs::Header: ...
geometry_msgs::Point: ...
geometry_msgs::Pose: ...
sensor_msgs::Imu: ...
sensor_msgs::LaserScan: ...

--- Custom Interface Messages ---
robot_interfaces::RobotStatus: ...
robot_interfaces::SensorReading: ...

--- Custom Service Types ---
SetModeRequest: ...
SetModeResponse: ...

--- Custom Action Types ---
NavigateGoal: ...
NavigateResult: ...
NavigateFeedback: ...

✓ All standard and custom interfaces loaded successfully!
```

## Verification Steps

1. **Package Discovery**:
   ```bash
   just list
   # Should show: robot_interfaces, robot_controller
   ```

2. **Build Everything**:
   ```bash
   just build
   # Should build both packages successfully
   ```

3. **Check Installation**:
   ```bash
   ls install/
   # Should have: robot_interfaces/, robot_controller/, setup.bash

   ls install/robot_controller/lib/robot_controller/
   # Should have: robot_controller (executable)
   ```

4. **Run Node**:
   ```bash
   just run
   # Should print all message types successfully
   ```

5. **Test Caching** (rebuild should be fast):
   ```bash
   just build-rust
   # Should use cached bindings, build in <5s
   ```

## Cleanup

```bash
just clean
```

## Troubleshooting

**Issue**: "Package not found: robot_interfaces"
- Solution: Build interfaces first: `just build-interfaces`

**Issue**: "cargo-ros2 not found" or "colcon doesn't recognize ament_cargo"
- Solution: Install both tools: `cd .. && just install`
