gtest_dep = dependency('gtest', main: true, disabler: true, required: false)
gmock_dep = dependency('gmock', disabler: true, required: false)
if not gtest_dep.found() or not gmock_dep.found()
    gtest_proj = import('cmake').subproject('googletest', required: false)
    if gtest_proj.found()
        gtest_dep = declare_dependency(
            dependencies: [
                dependency('threads'),
                gtest_proj.dependency('gtest'),
                gtest_proj.dependency('gtest_main'),
            ],
        )
        gmock_dep = gtest_proj.dependency('gmock')
    else
        assert(
            not get_option('tests').enabled(),
            'Googletest is required if tests are enabled',
        )
    endif
endif

test_base_lib = static_library(
    'test_base_lib',
    ['test_base.cpp'],
    include_directories: ['.', common_include],
    dependencies: [default_deps, modbus_rtu_dep],
)

test(
    'test_modbus_commands',
    executable(
        'test_modbus_commands',
        'test_modbus_commands.cpp',
        dependencies: [gtest_dep, gmock_dep, default_deps, modbus_rtu_dep],
        include_directories: ['.'],
    ),
)

test(
    'test_modbus',
    executable(
        'test_modbus',
        'test_modbus.cpp',
        'modbus_server_tester.cpp',
        link_with: [test_base_lib],
        dependencies: [gtest_dep, gmock_dep, default_deps, modbus_rtu_dep],
        include_directories: ['.'],
    ),
)

test(
    'test_port',
    executable(
        'test_port',
        'test_port.cpp',
        'modbus_server_tester.cpp',
        link_with: [test_base_lib],
        dependencies: [gtest_dep, gmock_dep, default_deps, modbus_rtu_dep],
        include_directories: ['.', common_include],
    ),
)

test(
    'test_inventory',
    executable(
        'test_inventory',
        'test_inventory.cpp',
        'modbus_server_tester.cpp',
        inventory_src,
        link_with: [test_base_lib],
        dependencies: [gtest_dep, gmock_dep, default_deps, modbus_rtu_dep],
        include_directories: ['.', common_include],
    ),
)

test(
    'test_sensors',
    executable(
        'test_sensors',
        'test_sensors.cpp',
        'modbus_server_tester.cpp',
        device_src,
        firmware_src,
        link_with: [modbus_common_lib, test_base_lib],
        dependencies: [gtest_dep, gmock_dep, default_deps, modbus_rtu_dep],
        include_directories: ['.', common_include],
    ),
)

test(
    'test_events',
    executable(
        'test_events',
        'test_events.cpp',
        link_with: [modbus_common_lib],
        dependencies: [gtest_dep, gmock_dep, default_deps, modbus_rtu_dep],
        include_directories: ['.', common_include],
    ),
)

test(
    'test_device_events',
    executable(
        'test_device_events',
        'test_device_events.cpp',
        'modbus_server_tester.cpp',
        device_src,
        firmware_src,
        link_with: [modbus_common_lib, test_base_lib],
        dependencies: [gtest_dep, gmock_dep, default_deps, modbus_rtu_dep],
        include_directories: ['.', common_include],
    ),
    is_parallel: false,  # To avoid conflict with test_events, which also simulates xyz.openbmc_project.Logging service operations
)

test(
    'test_firmware',
    executable(
        'test_firmware',
        'test_firmware.cpp',
        'modbus_server_tester.cpp',
        firmware_src,
        link_with: [modbus_common_lib, test_base_lib],
        dependencies: [gtest_dep, gmock_dep, default_deps, modbus_rtu_dep],
        include_directories: ['.', common_include],
    ),
)
