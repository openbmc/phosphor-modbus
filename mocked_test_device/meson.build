executable(
    'mock-modbus-server',
    ['mock_modbus_server.cpp'],
    include_directories: ['.', common_include],
    dependencies: [default_deps, modbus_rtu_dep],
    link_with: [modbus_rtu_lib],
    install: true,
    install_dir: get_option('libexecdir') / 'phosphor-modbus',
)
