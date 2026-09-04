# Everything but main, so the tests can link the same sources.
modbus_tool_src = files(
    'cmd/dump.cpp',
    'utils/common.cpp',
    'utils/entity_manager_lookup.cpp',
    'utils/json_writer.cpp',
    'utils/port_reservation.cpp',
    'utils/register_reader.cpp',
)

modbus_tool_include = include_directories('.')

executable(
    'modbus-tool',
    [
        'modbus_tool.cpp',
        modbus_tool_src,
        base_config_src,
        inventory_utils_src,
        profile_src,
    ],
    cpp_args: rtu_cpp_args,
    include_directories: ['.', common_include],
    dependencies: [cli11_dep, default_deps, modbus_rtu_dep],
    link_with: [modbus_common_lib],
    install: true,
    install_dir: get_option('bindir'),
)
