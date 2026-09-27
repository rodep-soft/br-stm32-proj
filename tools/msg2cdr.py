#!/usr/bin/env python3
"""
msg2cdr.py - Ultra-lightweight ROS 2 .msg to Micro-CDR C Header Generator

Usage:
    python3 msg2cdr.py --package <pkg_name> --msg <path/to/msg> --out-dir <out_dir>
    python3 msg2cdr.py --package <pkg_name> --msg-dir <dir_with_msgs> --out-dir <out_dir>
"""

import os
import sys
import re
import argparse
import hashlib
import json
from pathlib import Path

# Mapping ROS 2 primitive types to C types and Micro-CDR serialization suffixes
TYPE_MAP = {
    'bool': ('bool', '_bool'),
    'byte': ('uint8_t', '_uint8_t'),
    'char': ('char', '_char'),
    'int8': ('int8_t', '_int8_t'),
    'uint8': ('uint8_t', '_uint8_t'),
    'int16': ('int16_t', '_int16_t'),
    'uint16': ('uint16_t', '_uint16_t'),
    'int32': ('int32_t', '_int32_t'),
    'uint32': ('uint32_t', '_uint32_t'),
    'int64': ('int64_t', '_int64_t'),
    'uint64': ('uint64_t', '_uint64_t'),
    'float32': ('float', '_float'),
    'float64': ('double', '_double'),
}

# Mapping primitive types to printf format specifier and argument expression
PRINT_MAP = {
    'bool': ('%s', '({val} ? "true" : "false")'),
    'byte': ('0x%02X', '{val}'),
    'char': ("'%c'", '{val}'),
    'int8': ('%d', '{val}'),
    'uint8': ('%u', '{val}'),
    'int16': ('%d', '{val}'),
    'uint16': ('%u', '{val}'),
    'int32': ('%ld', '(long)({val})'),
    'uint32': ('%lu', '(unsigned long)({val})'),
    'int64': ('%lld', '(long long)({val})'),
    'uint64': ('%llu', '(unsigned long long)({val})'),
    'float32': ('%.2f', '(double)({val})'),
    'float64': ('%.2f', '{val}'),
}


class Field:
    def __init__(self, raw_type: str, name: str, array_size: int = None, default_val: str = None):
        self.raw_type = raw_type
        self.name = name
        self.array_size = array_size
        self.default_val = default_val

    @property
    def is_array(self) -> bool:
        return self.array_size is not None

    @property
    def is_string(self) -> bool:
        return self.raw_type == 'string'

    @property
    def is_header(self) -> bool:
        return self.raw_type in ('std_msgs/Header', 'std_msgs/msg/Header', 'Header')


def parse_msg_file(file_path: Path) -> tuple[list[Field], dict[str, str]]:
    fields = []
    metadata = {}
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            # Check for directive comments (e.g., # package: can_msgs or # type_hash: RIHS01_...)
            m_pkg = re.match(r'^\s*#\s*package\s*[:=]?\s*([a-zA-Z0-9_]+)', line)
            if m_pkg:
                metadata['package'] = m_pkg.group(1)
                continue
            m_hash = re.match(r'^\s*#\s*(?:type_)?hash\s*[:=]?\s*([a-zA-Z0-9_]+)', line)
            if m_hash:
                metadata['type_hash'] = m_hash.group(1)
                continue

            # Strip comments and extra whitespace
            line = re.sub(r'#.*$', '', line).strip()
            if not line:
                continue

            # Check for constant definitions (e.g., uint8 CONST_NAME=1)
            if '=' in line:
                # Constants are ignored in serialization payload
                continue

            parts = line.split()
            if len(parts) < 2:
                continue

            type_part = parts[0]
            name_part = parts[1]
            default_val = parts[2] if len(parts) > 2 else None

            # Array check: e.g. float32[3] or int32[10]
            array_match = re.match(r'^([a-zA-Z0-9_/]+)\[([0-9]+)\]$', type_part)
            if array_match:
                base_type = array_match.group(1)
                array_size = int(array_match.group(2))
                fields.append(Field(base_type, name_part, array_size=array_size, default_val=default_val))
            else:
                fields.append(Field(type_part, name_part, default_val=default_val))

    return fields, metadata


# ROS 2 REP-2011 type IDs: (scalar_id, array_id)
# See ROS 2 rosidl_generator_type_description
REP2011_TYPE_IDS = {
    'bool': (15, 63),
    'byte': (16, 64),
    'char': (13, 61),
    'float32': (10, 58),
    'float64': (11, 59),
    'int8': (2, 50),
    'uint8': (3, 51),
    'int16': (4, 52),
    'uint16': (5, 53),
    'int32': (6, 54),
    'uint32': (7, 55),
    'int64': (8, 56),
    'uint64': (9, 57),
    'string': (17, 65),
}

# Standard referenced types for Header
HEADER_REFERENCED_TYPES = [
    {
        'type_name': 'builtin_interfaces/msg/Time',
        'fields': [
            {'name': 'sec', 'type': {'type_id': 6, 'capacity': 0, 'string_capacity': 0, 'nested_type_name': ''}},
            {'name': 'nanosec', 'type': {'type_id': 7, 'capacity': 0, 'string_capacity': 0, 'nested_type_name': ''}}
        ]
    },
    {
        'type_name': 'std_msgs/msg/Header',
        'fields': [
            {'name': 'stamp', 'type': {'type_id': 1, 'capacity': 0, 'string_capacity': 0, 'nested_type_name': 'builtin_interfaces/msg/Time'}},
            {'name': 'frame_id', 'type': {'type_id': 17, 'capacity': 0, 'string_capacity': 0, 'nested_type_name': ''}}
        ]
    }
]


def compute_rihs01_hash(package_name: str, msg_name: str, fields: list[Field]) -> str:
    """Computes the official ROS 2 REP-2011 RIHS01 SHA-256 type hash."""
    type_name = f"{package_name}/msg/{msg_name}"
    field_entries = []
    has_header = False

    for f in fields:
        if f.is_header:
            has_header = True
            field_entries.append({
                'name': f.name,
                'type': {
                    'type_id': 1,
                    'capacity': 0,
                    'string_capacity': 0,
                    'nested_type_name': 'std_msgs/msg/Header'
                }
            })
        elif f.raw_type in REP2011_TYPE_IDS:
            scalar_id, array_id = REP2011_TYPE_IDS[f.raw_type]
            if f.is_array:
                field_entries.append({
                    'name': f.name,
                    'type': {
                        'type_id': array_id,
                        'capacity': f.array_size,
                        'string_capacity': 0,
                        'nested_type_name': ''
                    }
                })
            else:
                field_entries.append({
                    'name': f.name,
                    'type': {
                        'type_id': scalar_id,
                        'capacity': 0,
                        'string_capacity': 0,
                        'nested_type_name': ''
                    }
                })
        else:
            field_entries.append({
                'name': f.name,
                'type': {
                    'type_id': 1,
                    'capacity': 0,
                    'string_capacity': 0,
                    'nested_type_name': f.raw_type
                }
            })

    ref_types = HEADER_REFERENCED_TYPES if has_header else []
    ref_types_sorted = sorted(ref_types, key=lambda x: x['type_name'])

    hashable_dict = {
        'type_description': {
            'type_name': type_name,
            'fields': field_entries
        },
        'referenced_type_descriptions': ref_types_sorted
    }

    hashable_repr = json.dumps(hashable_dict, separators=(', ', ': '), sort_keys=False)
    sha = hashlib.sha256(hashable_repr.encode('utf-8')).hexdigest()
    return f"RIHS01_{sha}"


def generate_header(package_name: str, msg_file: Path, fields: list[Field], custom_hash: str = None) -> str:
    msg_name = msg_file.stem
    type_name = f"{package_name}_{msg_name}"
    guard_name = f"_{package_name.upper()}_{msg_name.upper()}_H_"
    type_hash = custom_hash or compute_rihs01_hash(package_name, msg_name, fields)
    dds_type_str = f"{package_name}::msg::dds_::{msg_name}_"

    # Struct fields definition
    struct_lines = []
    for f in fields:
        if f.is_header:
            struct_lines.append(f"""    struct {{
        int32_t sec;
        uint32_t nanosec;
        char frame_id[32];
    }} {f.name};""")
        elif f.is_string:
            struct_lines.append(f"    const char* {f.name};")
        elif f.raw_type in TYPE_MAP:
            c_type = TYPE_MAP[f.raw_type][0]
            if f.is_array:
                struct_lines.append(f"    {c_type} {f.name}[{f.array_size}];")
            else:
                struct_lines.append(f"    {c_type} {f.name};")
        else:
            struct_lines.append(f"    /* Unsupported type: {f.raw_type} */ {f.raw_type} {f.name};")

    struct_body = "\n".join(struct_lines)

    # Serialization statements
    ser_lines = []
    for f in fields:
        if f.is_header:
            ser_lines.append(f"""    ucdr_serialize_int32_t(ub, topic->{f.name}.sec);
    ucdr_serialize_uint32_t(ub, topic->{f.name}.nanosec);
    ucdr_serialize_string(ub, topic->{f.name}.frame_id);""")
        elif f.is_string:
            ser_lines.append(f"    ucdr_serialize_string(ub, topic->{f.name});")
        elif f.raw_type in TYPE_MAP:
            suffix = TYPE_MAP[f.raw_type][1]
            if f.is_array:
                ser_lines.append(f"    ucdr_serialize_array{suffix}(ub, topic->{f.name}, {f.array_size});")
            else:
                ser_lines.append(f"    ucdr_serialize{suffix}(ub, topic->{f.name});")

    ser_body = "\n".join(ser_lines)

    # Deserialization statements
    deser_lines = []
    for f in fields:
        if f.is_header:
            deser_lines.append(f"""    ucdr_deserialize_int32_t(ub, &topic->{f.name}.sec);
    ucdr_deserialize_uint32_t(ub, &topic->{f.name}.nanosec);
    ucdr_deserialize_string(ub, topic->{f.name}.frame_id, sizeof(topic->{f.name}.frame_id));""")
        elif f.is_string:
            deser_lines.append(f"    /* string deserialization requires buffer capacity */")
        elif f.raw_type in TYPE_MAP:
            suffix = TYPE_MAP[f.raw_type][1]
            if f.is_array:
                deser_lines.append(f"    ucdr_deserialize_array{suffix}(ub, topic->{f.name}, {f.array_size});")
            else:
                deser_lines.append(f"    ucdr_deserialize{suffix}(ub, &topic->{f.name});")

    deser_body = "\n".join(deser_lines)

    # Print function statements
    fmt_parts = []
    arg_parts = []
    for f in fields:
        if f.is_header:
            fmt_parts.append(f'{f.name}.frame_id=\\"%s\\"')
            arg_parts.append(f'topic->{f.name}.frame_id')
        elif f.is_string:
            fmt_parts.append(f'{f.name}=\\"%s\\"')
            arg_parts.append(f'topic->{f.name}')
        elif f.raw_type in PRINT_MAP:
            spec, expr_template = PRINT_MAP[f.raw_type]
            if f.is_array:
                arr_fmts = [spec] * f.array_size
                fmt_parts.append(f"{f.name}=[{', '.join(arr_fmts)}]")
                for idx in range(f.array_size):
                    arg_parts.append(expr_template.format(val=f'topic->{f.name}[{idx}]'))
            else:
                fmt_parts.append(f"{f.name}={spec}")
                arg_parts.append(expr_template.format(val=f'topic->{f.name}'))

    fmt_string = " ".join(fmt_parts)
    if arg_parts:
        args_formatted = ",\n        " + ",\n        ".join(arg_parts)
        print_body = f'    printf("[{msg_name}] {fmt_string}\\r\\n"{args_formatted});'
    else:
        print_body = f'    printf("[{msg_name}]\\r\\n");'

    content = f"""// Generated by msg2cdr.py for ROS 2 / Micro-CDR. DO NOT EDIT!
#ifndef {guard_name}
#define {guard_name}

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <ucdr/microcdr.h>

#ifdef __cplusplus
extern "C" {{
#endif

/* Type metadata */
#define {type_name}_PACKAGE "{package_name}"
#define {type_name}_MSG_NAME "{msg_name}"
#define {type_name}_DDS_TYPE "{dds_type_str}"
#define {type_name}_TYPE_HASH "{type_hash}"

/* Macro to generate Zenoh KeyExpr: <domain_id>/<topic_name>/<dds_type>/<type_hash> */
#define {type_name}_KEYEXPR(domain_id, topic_name) \\
    domain_id "/" topic_name "/" {type_name}_DDS_TYPE "/" {type_name}_TYPE_HASH

/* Message structure */
typedef struct {{
{struct_body}
}} {type_name};

/**
 * @brief Serialize {type_name} to CDR format with header
 */
static inline bool {type_name}_serialize(ucdrBuffer* ub, const {type_name}* topic) {{
    if (ub == NULL || topic == NULL) return false;

    // 1. CDR encapsulation header (Little Endian)
    ucdr_serialize_uint8_t(ub, 0x00);
    ucdr_serialize_uint8_t(ub, 0x01);
    ucdr_serialize_uint16_t(ub, 0x0000);

    // 2. Serialize fields
{ser_body}

    return !ucdr_buffer_has_error(ub);
}}

/**
 * @brief Deserialize {type_name} from CDR format with header
 */
static inline bool {type_name}_deserialize(ucdrBuffer* ub, {type_name}* topic) {{
    if (ub == NULL || topic == NULL) return false;

    // 1. Consume CDR encapsulation header
    uint8_t dummy8;
    uint16_t dummy16;
    ucdr_deserialize_uint8_t(ub, &dummy8);
    ucdr_deserialize_uint8_t(ub, &dummy8);
    ucdr_deserialize_uint16_t(ub, &dummy16);

    // 2. Deserialize fields
{deser_body}

    return !ucdr_buffer_has_error(ub);
}}

/**
 * @brief Print {type_name} to stdout / serial
 */
static inline void {type_name}_print(const {type_name}* topic) {{
    if (topic == NULL) return;
{print_body}
}}

#ifdef __cplusplus
}}
#endif

#endif // {guard_name}
"""
    return content


def main():
    parser = argparse.ArgumentParser(description="ROS 2 .msg to Micro-CDR C Header Generator")
    parser.add_argument("--package", required=True, help="Package name (e.g. robot_msgs)")
    parser.add_argument("--msg", help="Single .msg file path")
    parser.add_argument("--msg-dir", help="Directory containing .msg files")
    parser.add_argument("--out-dir", required=True, help="Output directory for generated headers")
    parser.add_argument("--type-hash", help="Override Type Hash (e.g. RIHS01_...)")

    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    msg_files = []
    if args.msg:
        msg_files.append(Path(args.msg))
    elif args.msg_dir:
        msg_files.extend(Path(args.msg_dir).glob("*.msg"))
    else:
        print("Error: Specify either --msg or --msg-dir", file=sys.stderr)
        sys.exit(1)

    if not msg_files:
        print("No .msg files found.", file=sys.stderr)
        sys.exit(1)

    for msg_file in msg_files:
        fields, metadata = parse_msg_file(msg_file)
        pkg = metadata.get('package', args.package)
        type_hash = metadata.get('type_hash', args.type_hash)
        header_code = generate_header(pkg, msg_file, fields, custom_hash=type_hash)
        out_file = out_dir / f"{msg_file.stem}.h"
        with open(out_file, 'w', encoding='utf-8') as f:
            f.write(header_code)
        print(f"[msg2cdr] Generated: {out_file} (package: {pkg}, fields: {len(fields)})")


if __name__ == "__main__":
    main()
