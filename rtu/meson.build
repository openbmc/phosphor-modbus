modbus_rtu_lib = static_library(
    'modbus_rtu_lib',
    [
        'modbus/modbus.cpp',
        'modbus/modbus_commands.cpp',
        'modbus/modbus_message.cpp',
    ],
    include_directories: ['.'],
    dependencies: [default_deps],
)

modbus_rtu_dep = declare_dependency(
    include_directories: ['.'],
    link_with: [modbus_rtu_lib],
    dependencies: [default_deps],
)

executable(
    'phosphor-modbus-rtu',
    ['device_manager.cpp', 'inventory_device.cpp', 'serial_port.cpp'],
    include_directories: ['.', common_include],
    dependencies: [default_deps],
    link_with: [modbus_common_lib, modbus_rtu_lib],
    install: true,
    install_dir: get_option('libexecdir') / 'phosphor-modbus',
)

executable(
    'modbus-test-server',
    ['TestServer.cpp'],
    include_directories: ['.', common_include],
    dependencies: [default_deps],
    link_with: [modbus_rtu_lib],
    install: true,
    install_dir: get_option('libexecdir') / 'phosphor-modbus',
)

executable(
    'modbus-test-usb',
    ['TestUSB.cpp'],
    include_directories: ['.'],
    dependencies: [default_deps],
    install: true,
    install_dir: get_option('libexecdir') / 'phosphor-modbus',
)

systemd_system_unit_dir = dependency('systemd').get_variable(
    'systemdsystemunitdir',
    pkgconfig_define: ['prefix', get_option('prefix')],
)

install_data(
    'xyz.openbmc_project.ModbusRTU.service',
    install_dir: systemd_system_unit_dir,
)
