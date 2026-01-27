#!/usr/bin/env python3
"""
nano-ros C++ Message Generator

Generates C++ message structs with CDR serialization from .msg files.
Wire-compatible with Rust nano-ros-serdes and ROS 2.

Usage:
    python3 nano_ros_generate_cpp.py <msg_file> <output_dir> [--package <name>]
    python3 nano_ros_generate_cpp.py <msg_file> <output_dir> [--package <name>] --embedded

Example:
    python3 nano_ros_generate_cpp.py msg/MyMessage.msg build/generated --package my_pkg
    python3 nano_ros_generate_cpp.py msg/MyMessage.msg build/generated --package my_pkg --embedded
"""

import argparse
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# Global flag for embedded mode
EMBEDDED_MODE = False

# Default buffer sizes for embedded mode
EMBEDDED_STRING_SIZE = 256
EMBEDDED_VECTOR_SIZE = 64

# ROS primitive types to C++ types (desktop mode)
PRIMITIVE_TYPES_DESKTOP = {
    'bool': 'bool',
    'byte': 'uint8_t',
    'char': 'char',
    'float32': 'float',
    'float64': 'double',
    'int8': 'int8_t',
    'uint8': 'uint8_t',
    'int16': 'int16_t',
    'uint16': 'uint16_t',
    'int32': 'int32_t',
    'uint32': 'uint32_t',
    'int64': 'int64_t',
    'uint64': 'uint64_t',
    'string': 'std::string',
}

# ROS primitive types to C++ types (embedded mode)
PRIMITIVE_TYPES_EMBEDDED = {
    'bool': 'bool',
    'byte': 'uint8_t',
    'char': 'char',
    'float32': 'float',
    'float64': 'double',
    'int8': 'int8_t',
    'uint8': 'uint8_t',
    'int16': 'int16_t',
    'uint16': 'uint16_t',
    'int32': 'int32_t',
    'uint32': 'uint32_t',
    'int64': 'int64_t',
    'uint64': 'uint64_t',
    'string': f'nano_ros::String<{EMBEDDED_STRING_SIZE}>',
}


def get_primitive_types():
    """Get the appropriate primitive types map based on mode."""
    return PRIMITIVE_TYPES_EMBEDDED if EMBEDDED_MODE else PRIMITIVE_TYPES_DESKTOP


# Alias for backwards compatibility
PRIMITIVE_TYPES = PRIMITIVE_TYPES_DESKTOP

# CDR write methods for each type
CDR_WRITE_METHODS = {
    'bool': 'write_bool',
    'byte': 'write_u8',
    'char': 'write_u8',  # char is serialized as uint8
    'float32': 'write_f32',
    'float64': 'write_f64',
    'int8': 'write_i8',
    'uint8': 'write_u8',
    'int16': 'write_i16',
    'uint16': 'write_u16',
    'int32': 'write_i32',
    'uint32': 'write_u32',
    'int64': 'write_i64',
    'uint64': 'write_u64',
    'string': 'write_string',
}

# CDR read methods for each type
CDR_READ_METHODS = {
    'bool': 'read_bool',
    'byte': 'read_u8',
    'char': 'read_u8',
    'float32': 'read_f32',
    'float64': 'read_f64',
    'int8': 'read_i8',
    'uint8': 'read_u8',
    'int16': 'read_i16',
    'uint16': 'read_u16',
    'int32': 'read_i32',
    'uint32': 'read_u32',
    'int64': 'read_i64',
    'uint64': 'read_u64',
    'string': 'read_string',
}

# Default values for primitives
DEFAULT_VALUES = {
    'bool': 'false',
    'byte': '0',
    'char': '0',
    'float32': '0.0f',
    'float64': '0.0',
    'int8': '0',
    'uint8': '0',
    'int16': '0',
    'uint16': '0',
    'int32': '0',
    'uint32': '0',
    'int64': '0',
    'uint64': '0',
    'string': '',  # std::string default ctor is fine
}


@dataclass
class Field:
    """Represents a message field."""
    ros_type: str
    name: str
    is_array: bool = False
    array_size: Optional[int] = None  # None for unbounded, number for fixed
    default_value: Optional[str] = None
    is_nested: bool = False
    nested_package: Optional[str] = None


@dataclass
class Constant:
    """Represents a message constant."""
    ros_type: str
    name: str
    value: str


def parse_msg_file(msg_path: Path) -> tuple[list[Field], list[Constant]]:
    """Parse a .msg file and return fields and constants."""
    fields = []
    constants = []

    with open(msg_path) as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()

            # Skip empty lines and comments
            if not line or line.startswith('#'):
                continue

            # Check for constant definition (TYPE NAME=VALUE)
            const_match = re.match(r'^(\w+)\s+(\w+)\s*=\s*(.+)$', line)
            if const_match:
                ros_type, name, value = const_match.groups()
                constants.append(Constant(ros_type, name, value.strip()))
                continue

            # Parse field definition
            field = parse_field(line, line_num)
            if field:
                fields.append(field)

    return fields, constants


@dataclass
class ServiceDefinition:
    """Represents a parsed service definition."""
    request_fields: list[Field]
    request_constants: list[Constant]
    response_fields: list[Field]
    response_constants: list[Constant]


def parse_srv_file(srv_path: Path) -> ServiceDefinition:
    """Parse a .srv file and return request and response fields."""
    request_fields = []
    request_constants = []
    response_fields = []
    response_constants = []

    in_response = False

    with open(srv_path) as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()

            # Check for separator
            if line == '---':
                in_response = True
                continue

            # Skip empty lines and comments
            if not line or line.startswith('#'):
                continue

            # Check for constant definition (TYPE NAME=VALUE)
            const_match = re.match(r'^(\w+)\s+(\w+)\s*=\s*(.+)$', line)
            if const_match:
                ros_type, name, value = const_match.groups()
                const = Constant(ros_type, name, value.strip())
                if in_response:
                    response_constants.append(const)
                else:
                    request_constants.append(const)
                continue

            # Parse field definition
            field = parse_field(line, line_num)
            if field:
                if in_response:
                    response_fields.append(field)
                else:
                    request_fields.append(field)

    return ServiceDefinition(
        request_fields=request_fields,
        request_constants=request_constants,
        response_fields=response_fields,
        response_constants=response_constants,
    )


def parse_field(line: str, line_num: int) -> Optional[Field]:
    """Parse a single field definition line."""
    # Match: TYPE[SIZE] NAME [DEFAULT]
    # Examples:
    #   int32 data
    #   string[] names
    #   float64[3] position
    #   uint8[<=100] bounded_data
    #   geometry_msgs/Point position

    # Split into type and name (and optional default)
    parts = line.split(None, 2)
    if len(parts) < 2:
        print(f"Warning: line {line_num}: cannot parse field: {line}", file=sys.stderr)
        return None

    type_str = parts[0]
    name = parts[1]
    default_value = parts[2] if len(parts) > 2 else None

    # Parse array notation
    is_array = False
    array_size = None

    array_match = re.match(r'^(.+)\[([^\]]*)\]$', type_str)
    if array_match:
        type_str = array_match.group(1)
        size_str = array_match.group(2)
        is_array = True

        if size_str == '':
            # Unbounded array: type[]
            array_size = None
        elif size_str.startswith('<='):
            # Bounded array: type[<=N] - treat as unbounded for now
            array_size = None
        else:
            # Fixed array: type[N]
            try:
                array_size = int(size_str)
            except ValueError:
                print(f"Warning: line {line_num}: invalid array size: {size_str}", file=sys.stderr)
                array_size = None

    # Check for nested type (package/Type)
    is_nested = False
    nested_package = None

    if '/' in type_str:
        parts = type_str.split('/')
        if len(parts) == 2:
            nested_package = parts[0]
            type_str = parts[1]
            is_nested = True
    elif type_str not in PRIMITIVE_TYPES:
        # Local nested type (same package)
        is_nested = True

    return Field(
        ros_type=type_str,
        name=name,
        is_array=is_array,
        array_size=array_size,
        default_value=default_value,
        is_nested=is_nested,
        nested_package=nested_package,
    )


def to_cpp_type(field: Field, current_package: str) -> str:
    """Convert a field to its C++ type."""
    primitive_types = get_primitive_types()

    if field.is_nested:
        if field.nested_package:
            base_type = f"{field.nested_package}::msg::{field.ros_type}"
        else:
            base_type = field.ros_type
    else:
        base_type = primitive_types.get(field.ros_type, field.ros_type)

    if field.is_array:
        if field.array_size is not None:
            # Fixed-size array: std::array for both modes
            return f"std::array<{base_type}, {field.array_size}>"
        # Variable-size sequence
        if EMBEDDED_MODE:
            return f"nano_ros::Vector<{base_type}, {EMBEDDED_VECTOR_SIZE}>"
        return f"std::vector<{base_type}>"

    return base_type


def generate_serialize(field: Field, current_package: str) -> str:
    """Generate serialization code for a field."""
    name = field.name

    if field.is_array:
        if field.array_size is not None:
            # Fixed-size array
            if field.is_nested:
                return f"for (const auto& elem : {name}) {{ elem.serialize(writer); }}"
            write_method = CDR_WRITE_METHODS.get(field.ros_type, 'write_u8')
            return f"for (const auto& elem : {name}) {{ writer.{write_method}(elem); }}"
        else:
            # Variable-size sequence
            if field.is_nested:
                return (f"writer.write_sequence_length(static_cast<uint32_t>({name}.size()));\n"
                        f"        for (const auto& elem : {name}) {{ elem.serialize(writer); }}")
            # Use template write_sequence for primitives
            return f"writer.write_sequence({name})"
    else:
        if field.is_nested:
            return f"{name}.serialize(writer)"
        write_method = CDR_WRITE_METHODS.get(field.ros_type)
        if write_method:
            return f"writer.{write_method}({name})"
        return f"// TODO: unknown type {field.ros_type}"


def generate_deserialize(field: Field, current_package: str) -> str:
    """Generate deserialization code for a field."""
    name = field.name

    if field.is_array:
        if field.array_size is not None:
            # Fixed-size array
            if field.is_nested:
                return f"for (auto& elem : {name}) {{ elem.deserialize(reader); }}"
            # For embedded mode strings in arrays, use output parameter
            if EMBEDDED_MODE and field.ros_type == 'string':
                return f"for (auto& elem : {name}) {{ reader.read_string(elem); }}"
            read_method = CDR_READ_METHODS.get(field.ros_type, 'read_u8')
            return f"for (auto& elem : {name}) {{ elem = reader.{read_method}(); }}"
        else:
            # Variable-size sequence
            if field.is_nested:
                cpp_type = to_cpp_type(Field(field.ros_type, '', is_nested=True, nested_package=field.nested_package), current_package)
                return (f"{name}.resize(reader.read_sequence_length());\n"
                        f"        for (auto& elem : {name}) {{ elem.deserialize(reader); }}")
            # For embedded mode, handle string sequences specially
            if EMBEDDED_MODE and field.ros_type == 'string':
                return (f"{{ auto len = reader.read_u32(); {name}.clear();\n"
                        f"          for (uint32_t i = 0; i < len; ++i) {{ "
                        f"nano_ros::String<{EMBEDDED_STRING_SIZE}> s; reader.read_string(s); {name}.push_back(s); }} }}")
            # Use template read_sequence for primitives
            primitive_types = get_primitive_types()
            cpp_type = primitive_types.get(field.ros_type, field.ros_type)
            return f"{name} = reader.read_sequence<{cpp_type}>()"
    else:
        if field.is_nested:
            return f"{name}.deserialize(reader)"
        # For embedded mode strings, use output parameter
        if EMBEDDED_MODE and field.ros_type == 'string':
            return f"reader.read_string({name})"
        read_method = CDR_READ_METHODS.get(field.ros_type)
        if read_method:
            return f"{name} = reader.{read_method}()"
        return f"// TODO: unknown type {field.ros_type}"


def generate_default_value(field: Field) -> str:
    """Generate default value for a field."""
    if field.default_value:
        return f"{{{field.default_value}}}"

    if field.is_array:
        return "{}"  # Empty vector/array

    default = DEFAULT_VALUES.get(field.ros_type)
    if default is not None and default != '':
        return f"{{{default}}}"

    return "{}"


def msg_name_to_header_name(msg_name: str) -> str:
    """Convert MessageName to message_name.hpp style.

    Examples:
        Int32 -> int32.hpp
        UInt32 -> uint32.hpp
        MyMessage -> my_message.hpp
        Float64 -> float64.hpp
    """
    # Handle common patterns where we don't want underscores
    # UInt32 -> uint32 (not u_int32)
    # Int32 -> int32 (not int_32)
    result = msg_name

    # Insert underscore only between lowercase-uppercase transitions
    # This handles: MyMessage -> my_message, but not UInt32 -> u_int32
    result = re.sub(r'([a-z])([A-Z])', r'\1_\2', result)

    return result.lower() + '.hpp'


def generate_cpp_header(msg_name: str, package: str, fields: list[Field],
                        constants: list[Constant], dependencies: set[str]) -> str:
    """Generate the C++ header file content."""
    header_guard = f"{package.upper()}_MSG_{msg_name.upper()}_HPP"
    namespace = f"{package}::msg"
    type_name = f"{package}/msg/{msg_name}"

    # Collect includes based on mode
    if EMBEDDED_MODE:
        includes = [
            '#include <nano_ros/embedded/cdr.hpp>',
            '#include <nano_ros/embedded/types.hpp>',
        ]
    else:
        includes = ['#include <nano_ros/cdr.hpp>']

    std_includes = set()

    for field in fields:
        cpp_type = to_cpp_type(field, package)
        if 'std::vector' in cpp_type:
            std_includes.add('#include <vector>')
        if 'std::array' in cpp_type:
            std_includes.add('#include <array>')
        if 'std::string' in cpp_type and not EMBEDDED_MODE:
            std_includes.add('#include <string>')

        # Add include for nested types
        if field.is_nested:
            if field.nested_package:
                dep_header = msg_name_to_header_name(field.ros_type)
                includes.append(f'#include <{field.nested_package}/msg/{dep_header}>')
            else:
                dep_header = msg_name_to_header_name(field.ros_type)
                includes.append(f'#include "{dep_header}"')

    includes.extend(sorted(std_includes))
    includes.append('#include <cstdint>')

    # Generate field declarations
    field_decls = []
    for field in fields:
        cpp_type = to_cpp_type(field, package)
        default = generate_default_value(field)
        field_decls.append(f"    {cpp_type} {field.name}{default};")

    # Generate constant declarations
    const_decls = []
    for const in constants:
        cpp_type = PRIMITIVE_TYPES.get(const.ros_type, const.ros_type)
        const_decls.append(f"    static constexpr {cpp_type} {const.name} = {const.value};")

    # Generate serialize body
    serialize_lines = []
    for field in fields:
        serialize_lines.append(f"        {generate_serialize(field, package)};")

    # Generate deserialize body
    deserialize_lines = []
    for field in fields:
        deserialize_lines.append(f"        {generate_deserialize(field, package)};")

    # CDR type names based on mode
    cdr_writer = 'nano_ros::EmbeddedCdrWriter' if EMBEDDED_MODE else 'nano_ros::CdrWriter'
    cdr_reader = 'nano_ros::EmbeddedCdrReader' if EMBEDDED_MODE else 'nano_ros::CdrReader'

    # Build the header
    header = f"""#pragma once

// Generated by nano_ros_generate_cpp.py - DO NOT EDIT

{chr(10).join(includes)}

namespace {namespace} {{

struct {msg_name} {{
{chr(10).join(const_decls)}
{chr(10).join(field_decls) if const_decls else chr(10).join(field_decls)}

    /// Serialize to CDR format
    void serialize({cdr_writer}& writer) const {{
{chr(10).join(serialize_lines) if serialize_lines else '        // No fields'}
    }}

    /// Deserialize from CDR format
    void deserialize({cdr_reader}& reader) {{
{chr(10).join(deserialize_lines) if deserialize_lines else '        // No fields'}
    }}

    /// Get the ROS type name
    static constexpr const char* type_name() {{
        return "{type_name}";
    }}
}};

}}  // namespace {namespace}
"""
    return header


def generate_message(msg_path: Path, output_dir: Path, package: str) -> Path:
    """Generate C++ header from .msg file."""
    # Parse the message file
    fields, constants = parse_msg_file(msg_path)

    # Extract message name from filename
    msg_name = msg_path.stem

    # Collect dependencies
    dependencies = set()
    for field in fields:
        if field.is_nested and field.nested_package:
            dependencies.add(field.nested_package)

    # Generate header content
    header_content = generate_cpp_header(msg_name, package, fields, constants, dependencies)

    # Create output directory
    msg_output_dir = output_dir / package / 'msg'
    msg_output_dir.mkdir(parents=True, exist_ok=True)

    # Write header file
    header_name = msg_name_to_header_name(msg_name)
    output_path = msg_output_dir / header_name
    output_path.write_text(header_content)

    return output_path


def generate_struct_body(struct_name: str, fields: list[Field], constants: list[Constant],
                         package: str, type_name: str) -> str:
    """Generate a struct body with fields, serialize, and deserialize methods."""
    primitive_types = get_primitive_types()

    # Generate field declarations
    field_decls = []
    for field in fields:
        cpp_type = to_cpp_type(field, package)
        default = generate_default_value(field)
        field_decls.append(f"        {cpp_type} {field.name}{default};")

    # Generate constant declarations
    const_decls = []
    for const in constants:
        cpp_type = primitive_types.get(const.ros_type, const.ros_type)
        const_decls.append(f"        static constexpr {cpp_type} {const.name} = {const.value};")

    # Generate serialize body
    serialize_lines = []
    for field in fields:
        serialize_lines.append(f"            {generate_serialize(field, package)};")

    # Generate deserialize body
    deserialize_lines = []
    for field in fields:
        deserialize_lines.append(f"            {generate_deserialize(field, package)};")

    # CDR type names based on mode
    cdr_writer = 'nano_ros::EmbeddedCdrWriter' if EMBEDDED_MODE else 'nano_ros::CdrWriter'
    cdr_reader = 'nano_ros::EmbeddedCdrReader' if EMBEDDED_MODE else 'nano_ros::CdrReader'

    return f"""    struct {struct_name} {{
{chr(10).join(const_decls) + chr(10) if const_decls else ''}{chr(10).join(field_decls)}

        /// Serialize to CDR format
        void serialize({cdr_writer}& writer) const {{
{chr(10).join(serialize_lines) if serialize_lines else '            // No fields'}
        }}

        /// Deserialize from CDR format
        void deserialize({cdr_reader}& reader) {{
{chr(10).join(deserialize_lines) if deserialize_lines else '            // No fields'}
        }}

        /// Get the ROS type name
        static constexpr const char* type_name() {{
            return "{type_name}";
        }}
    }};"""


def generate_srv_header(srv_name: str, package: str, srv_def: ServiceDefinition,
                        dependencies: set[str]) -> str:
    """Generate the C++ header file content for a service."""
    header_guard = f"{package.upper()}_SRV_{srv_name.upper()}_HPP"
    namespace = f"{package}::srv"
    request_type_name = f"{package}/srv/{srv_name}_Request"
    response_type_name = f"{package}/srv/{srv_name}_Response"

    # Collect includes based on mode
    if EMBEDDED_MODE:
        includes = [
            '#include <nano_ros/embedded/cdr.hpp>',
            '#include <nano_ros/embedded/types.hpp>',
        ]
    else:
        includes = ['#include <nano_ros/cdr.hpp>']

    std_includes = set()

    all_fields = srv_def.request_fields + srv_def.response_fields
    for field in all_fields:
        cpp_type = to_cpp_type(field, package)
        if 'std::vector' in cpp_type:
            std_includes.add('#include <vector>')
        if 'std::array' in cpp_type:
            std_includes.add('#include <array>')
        if 'std::string' in cpp_type and not EMBEDDED_MODE:
            std_includes.add('#include <string>')

        # Add include for nested types
        if field.is_nested:
            if field.nested_package:
                dep_header = msg_name_to_header_name(field.ros_type)
                includes.append(f'#include <{field.nested_package}/msg/{dep_header}>')
            else:
                dep_header = msg_name_to_header_name(field.ros_type)
                includes.append(f'#include "{dep_header}"')

    includes.extend(sorted(std_includes))
    includes.append('#include <cstdint>')

    # Generate Request struct body
    request_body = generate_struct_body('Request', srv_def.request_fields,
                                        srv_def.request_constants, package, request_type_name)

    # Generate Response struct body
    response_body = generate_struct_body('Response', srv_def.response_fields,
                                         srv_def.response_constants, package, response_type_name)

    # Build the header
    header = f"""#pragma once

// Generated by nano_ros_generate_cpp.py - DO NOT EDIT

{chr(10).join(includes)}

namespace {namespace} {{

/// Service definition for {package}/srv/{srv_name}
struct {srv_name} {{
{request_body}

{response_body}
}};

}}  // namespace {namespace}
"""
    return header


def generate_service(srv_path: Path, output_dir: Path, package: str) -> Path:
    """Generate C++ header from .srv file."""
    # Parse the service file
    srv_def = parse_srv_file(srv_path)

    # Extract service name from filename
    srv_name = srv_path.stem

    # Collect dependencies
    dependencies = set()
    for field in srv_def.request_fields + srv_def.response_fields:
        if field.is_nested and field.nested_package:
            dependencies.add(field.nested_package)

    # Generate header content
    header_content = generate_srv_header(srv_name, package, srv_def, dependencies)

    # Create output directory
    srv_output_dir = output_dir / package / 'srv'
    srv_output_dir.mkdir(parents=True, exist_ok=True)

    # Write header file
    header_name = msg_name_to_header_name(srv_name)
    output_path = srv_output_dir / header_name
    output_path.write_text(header_content)

    return output_path


def main():
    parser = argparse.ArgumentParser(
        description='Generate C++ message/service types from .msg/.srv files'
    )
    parser.add_argument('input_file', type=Path, help='Input .msg or .srv file')
    parser.add_argument('output_dir', type=Path, help='Output directory')
    parser.add_argument('--package', '-p', default='my_package',
                        help='Package name (default: my_package)')
    parser.add_argument('--embedded', '-e', action='store_true',
                        help='Generate for embedded systems (no std library)')
    parser.add_argument('--string-size', type=int, default=256,
                        help='Maximum string size for embedded mode (default: 256)')
    parser.add_argument('--vector-size', type=int, default=64,
                        help='Maximum vector size for embedded mode (default: 64)')

    args = parser.parse_args()

    # Set embedded mode globals
    global EMBEDDED_MODE, EMBEDDED_STRING_SIZE, EMBEDDED_VECTOR_SIZE
    EMBEDDED_MODE = args.embedded
    EMBEDDED_STRING_SIZE = args.string_size
    EMBEDDED_VECTOR_SIZE = args.vector_size

    # Update embedded string type with configured size
    if EMBEDDED_MODE:
        PRIMITIVE_TYPES_EMBEDDED['string'] = f'nano_ros::String<{EMBEDDED_STRING_SIZE}>'

    if not args.input_file.exists():
        print(f"Error: {args.input_file} not found", file=sys.stderr)
        sys.exit(1)

    # Check file extension and generate appropriate output
    suffix = args.input_file.suffix.lower()
    if suffix == '.srv':
        output_path = generate_service(args.input_file, args.output_dir, args.package)
    elif suffix == '.msg':
        output_path = generate_message(args.input_file, args.output_dir, args.package)
    else:
        print(f"Error: Unknown file type: {suffix} (expected .msg or .srv)", file=sys.stderr)
        sys.exit(1)

    print(f"Generated: {output_path}")


if __name__ == '__main__':
    main()
