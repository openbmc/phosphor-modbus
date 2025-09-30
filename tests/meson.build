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

test(
    'test_modbus_commands',
    executable(
        'test_modbus_commands',
        'test_modbus_commands.cpp',
        cpp_args: ['-fext-numeric-literals'],
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
        dependencies: [gtest_dep, gmock_dep, default_deps, modbus_rtu_dep],
        include_directories: ['.', common_include],
    ),
)

executable(
    'modbus-test-server',
    ['modbus_test_server.cpp'],
    include_directories: ['.', common_include],
    dependencies: [default_deps, modbus_rtu_dep],
    link_with: [modbus_rtu_lib],
    install: true,
    install_dir: get_option('libexecdir') / 'phosphor-modbus',
)
