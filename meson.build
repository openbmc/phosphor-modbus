project(
    'phosphor-modbus',
    'cpp',
    default_options: ['warning_level=3', 'werror=true', 'cpp_std=c++23'],
    license: 'Apache-2.0',
    version: '0.1',
    meson_version: '>=1.1.1',
)

nlohmann_json_dep = dependency('nlohmann_json', include_type: 'system')
phosphor_dbus_interfaces_dep = dependency('phosphor-dbus-interfaces')
phosphor_logging_dep = dependency('phosphor-logging')
sdbusplus_dep = dependency('sdbusplus')

default_deps = [
    nlohmann_json_dep,
    phosphor_dbus_interfaces_dep,
    phosphor_logging_dep,
    sdbusplus_dep,
]

common_include = include_directories('.')

subdir('common')

if get_option('modbus-rtu').allowed()
    subdir('rtu')
endif

if get_option('mocked-test-device').allowed()
    subdir('mocked_test_device')
endif

if get_option('tests').allowed()
    subdir('tests')
endif

profile_json_files = files(
    'rtu/profiles/Artesyn7000552531000PowerShelf.json',
    'rtu/profiles/Danfoss003Z8540Valve.json',
    'rtu/profiles/DeltaBBUBS723AE000PowerShelf.json',
    'rtu/profiles/DeltaECD68000047PowerShelf.json',
    'rtu/profiles/DeltaECD68000049PowerShelf.json',
    'rtu/profiles/DeltaRDF040DSS5193E0ReservoirPumpUnit.json',
    'rtu/profiles/PanasonicBJBSB103APowerShelf.json',
)

install_data(
    profile_json_files,
    install_dir: get_option('datadir') / 'phosphor-modbus' / 'profiles',
)

custom_target(
    'validate_profiles',
    command: [
        find_program('python3'),
        files('scripts/validate_profiles.py'),
        files('rtu/schemas/device_profile.json'),
        profile_json_files,
    ],
    build_by_default: true,
    output: 'validate_profiles.log',
)
