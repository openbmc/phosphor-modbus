# Everything but main, so the tests can link the same sources.
modbus_tool_src = files(
    'cmd/dump.cpp',
    'cmd/write.cpp',
    'utils/common.cpp',
    'utils/entity_manager_lookup.cpp',
    'utils/json_writer.cpp',
    'utils/port_reservation.cpp',
    'utils/register_reader.cpp',
    'utils/serial_session.cpp',
)

modbus_tool_include = include_directories('.')

# A dump reports the version its schema declares, so the two cannot disagree.
# Reading the schema through fs makes a change to it reconfigure the build.
fs = import('fs')
dump_schema_version = run_command(
    find_program('python3'),
    '-c',
    'import json, sys; print(json.loads(sys.argv[1])["version"])',
    fs.read('schemas/dump.json'),
    check: true,
).stdout().strip()

modbus_tool_cpp_args = rtu_cpp_args + [
    '-DDUMP_SCHEMA_VERSION="' + dump_schema_version + '"',
]

executable(
    'modbus-tool',
    [
        'modbus_tool.cpp',
        modbus_tool_src,
        allowed_devices_src,
        base_config_src,
        device_utils_src,
        firmware_utils_src,
        inventory_utils_src,
        profile_src,
    ],
    cpp_args: modbus_tool_cpp_args,
    include_directories: ['.', common_include],
    dependencies: [cli11_dep, default_deps, modbus_rtu_dep],
    link_with: [modbus_common_lib],
    install: true,
    install_dir: get_option('bindir'),
)
