#!/usr/bin/env python3
"""Check the type name format used by ROS 2."""

import sys
sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')
sys.path.insert(0, '/opt/ros/humble/local/lib/python3.10/dist-packages')

from std_msgs.msg import Int32, String

# Check if we can get the DDS type name
print("std_msgs.msg.Int32:")
print(f"  __module__: {Int32.__module__}")
print(f"  __name__: {Int32.__name__}")
print(f"  __qualname__: {Int32.__qualname__}")

# Try to get rosidl info
try:
    from rosidl_runtime_py import get_message_slot_types
    print(f"  slot_types: {get_message_slot_types(Int32)}")
except:
    pass

# Try to find type support
try:
    from rosidl_generator_c import get_msg_type_support
    print(f"  type_support: available")
except:
    pass

print()
print("Looking for type name string...")

# The format rmw_zenoh uses is typically from the introspection type support
# Let's check what typesupport_introspection returns
try:
    from rosidl_typesupport_introspection_cpp import get_message_type_support_handle
    print("Introspection support available")
except ImportError:
    print("Introspection support not available via Python")

# Check the message structure
print()
print("Int32 message fields:")
print(f"  SLOT_TYPES: {getattr(Int32, 'SLOT_TYPES', 'N/A')}")
print(f"  _fields_and_field_types: {getattr(Int32, '_fields_and_field_types', 'N/A')}")

# The DDS type name is typically: <package>::msg::dds_::<MsgName>_
print()
print("Expected DDS type names:")
print("  std_msgs::msg::dds_::Int32_")
print("  std_msgs::msg::dds_::String_")
