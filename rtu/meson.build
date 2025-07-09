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
