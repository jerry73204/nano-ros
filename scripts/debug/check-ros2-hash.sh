#!/bin/bash
# Check what RIHS01 hash ROS 2 uses for std_msgs/msg/Int32

echo "=== ROS 2 Type Hash Check ==="
echo ""

# Source ROS 2 environment if available
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
else
    echo "ROS 2 not found"
    exit 1
fi

echo "ROS 2 distro: $ROS_DISTRO"
echo ""

# Get type hash info for Int32
echo "=== Type Info for std_msgs/msg/Int32 ==="
ros2 interface show std_msgs/msg/Int32

echo ""
echo "=== Type Hash (if available via interface info) ==="
# Try to get type hash using ros2cli or rosidl tools
python3 -c "
import hashlib
import json

# Type description for Int32 in the format used by RIHS01
# Based on rosidl type description format
type_desc = {
    'type_description': {
        'type_name': 'std_msgs/msg/Int32',
        'fields': [
            {'name': 'data', 'type': {'type_id': 5, 'capacity': 0}}
        ]
    },
    'referenced_type_descriptions': []
}

# Serialize to JSON (compact, no spaces)
json_str = json.dumps(type_desc, separators=(',', ':'))
print('JSON:', json_str)

# Compute SHA-256
hash_obj = hashlib.sha256(json_str.encode('utf-8'))
print('SHA-256:', hash_obj.hexdigest())
"

echo ""
echo "=== nano-ros Int32 hash ==="
echo "b6578ded3c58c626cfe8d1a6fb6e04f706f97e9f03d2727c9ff4e74b1cef0deb"

echo ""
echo "=== Check if they match ==="
