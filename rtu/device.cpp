#include "device.hpp"

#include "common/entity_manager_interface.hpp"

#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Inventory/Item/client.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <numeric>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

namespace config
{

using BasicVariantType =
    std::variant<std::vector<std::string>, std::vector<uint8_t>, std::string,
                 int64_t, uint64_t, double, int32_t, uint32_t, int16_t,
                 uint16_t, uint8_t, bool>;
using InventoryBaseConfigMap =
    boost::container::flat_map<std::string, BasicVariantType>;
using InventoryData =
    boost::container::flat_map<std::string, InventoryBaseConfigMap>;
using ManagedObjectType =
    boost::container::flat_map<sdbusplus::message::object_path, InventoryData>;

static const std::unordered_map<std::string_view,
                                std::pair<DeviceType, DeviceModel>>
    validDevices = {
        {ModbusRDF040DSS5193E0ReservoirPumpUnitInterface,
         {DeviceType::reservoirPumpUnit, DeviceModel::RDF040DSS5193E0}},
        {ModbusRDF040DSS5193E0HeatExchangerInterface,
         {DeviceType::heatExchanger, DeviceModel::RDF040DSS5193E0}}};

static auto processDeviceInterface(Config& config,
                                   const InventoryBaseConfigMap& configMap,
                                   const std::string& interfaceName) -> bool
{
    info("Processing device config");

    auto nameIter = configMap.find("Name");
    if (nameIter == configMap.end())
    {
        error("Missing Name in Device config");
        return false;
    }
    config.name = std::get<std::string>(nameIter->second);

    auto addressIter = configMap.find("Address");
    if (addressIter == configMap.end())
    {
        error("Missing Address for {NAME}", "NAME", config.name);
        return false;
    }
    config.address = std::get<uint64_t>(addressIter->second);

    auto portIter = configMap.find("SerialPort");
    if (portIter == configMap.end())
    {
        error("Missing SerialPort for {NAME}", "NAME", config.name);
        return false;
    }
    config.portName = std::get<std::string>(portIter->second);

    auto typeIter = configMap.find("Type");
    if (typeIter == configMap.end())
    {
        error("Missing Type for {NAME}", "NAME", config.name);
        return false;
    }

    for (const auto& [deviceInterface, deviceDetails] : validDevices)
    {
        if (interfaceName == deviceInterface)
        {
            config.deviceType = deviceDetails.first;
            config.deviceModel = deviceDetails.second;
            return true;
        }
    }

    error("Invalid device type for {NAME}", "NAME", config.name);
    return false;
}

static const auto sensorTypes = std::unordered_map<
    std::string_view, std::pair<std::string_view, SensorValueIntf::Unit>>{
    {"FanTach",
     {SensorValueIntf::namespace_path::fan_tach, SensorValueIntf::Unit::RPMS}},
    //{"LiquidLevel",
    {"LiquidFlow",
     {SensorValueIntf::namespace_path::liquidflow, SensorValueIntf::Unit::LPM}},
    {"Power",
     {SensorValueIntf::namespace_path::power, SensorValueIntf::Unit::Watts}},
    {"Pressure",
     {SensorValueIntf::namespace_path::pressure,
      SensorValueIntf::Unit::Pascals}},
    {"Temperature",
     {SensorValueIntf::namespace_path::temperature,
      SensorValueIntf::Unit::DegreesC}},
    //{"Volume", SensorType::volume}
};

static const auto formatTypes =
    std::unordered_map<std::string_view, SensorFormat>{
        {"Integer", SensorFormat::integer},
        {"Float", SensorFormat::floatingPoint}};

static auto processRegisterType(SensorRegister& sensorRegister,
                                const InventoryBaseConfigMap& configMap) -> bool
{
    auto typeIter = configMap.find("RegisterType");
    if (typeIter == configMap.end())
    {
        error("Missing RegisterType for {NAME}", "NAME", sensorRegister.name);
        return false;
    }
    auto type = sensorTypes.find(std::get<std::string>(typeIter->second));
    if (type == sensorTypes.end())
    {
        error("Invalid RegisterType for {NAME}", "NAME", sensorRegister.name);
        return false;
    }
    sensorRegister.pathSuffix = type->second.first;
    sensorRegister.unit = type->second.second;

    return true;
}

static auto processRegisterFormat(SensorRegister& sensorRegister,
                                  const InventoryBaseConfigMap& configMap)
    -> bool
{
    auto formatIter = configMap.find("Format");
    if (formatIter == configMap.end())
    {
        error("Missing Format for {NAME}", "NAME", sensorRegister.name);
        return false;
    }
    auto format = formatTypes.find(std::get<std::string>(formatIter->second));
    if (format == formatTypes.end())
    {
        error("Invalid Format for {NAME}", "NAME", sensorRegister.name);
        return false;
    }
    sensorRegister.format = format->second;

    return true;
}

static auto processSensorRegistersInterface(
    Config& config, const InventoryBaseConfigMap& configMap) -> bool
{
    SensorRegister sensorRegister = {};
    auto nameIter = configMap.find("Name");
    if (nameIter == configMap.end())
    {
        error("Missing Name for {NAME}", "NAME", config.name);
        return false;
    }
    sensorRegister.name = std::get<std::string>(nameIter->second);

    if (!processRegisterType(sensorRegister, configMap))
    {
        return false;
    }

    auto address = configMap.find("Address");
    if (address == configMap.end())
    {
        error("Missing Address for {NAME}", "NAME", sensorRegister.name);
        return false;
    }
    sensorRegister.offset = std::get<uint64_t>(address->second);

    auto sizeIter = configMap.find("Size");
    if (sizeIter == configMap.end())
    {
        error("Missing Size for {NAME}", "NAME", sensorRegister.name);
        return false;
    }
    sensorRegister.size = std::get<uint64_t>(sizeIter->second);

    auto precisionIter = configMap.find("Precision");
    if (precisionIter == configMap.end())
    {
        error("Missing Precision for {NAME}", "NAME", sensorRegister.name);
        return false;
    }
    sensorRegister.size = std::get<uint64_t>(precisionIter->second);

    auto scaleIter = configMap.find("Scale");
    if (scaleIter == configMap.end())
    {
        error("Missing Scale for {NAME}", "NAME", sensorRegister.name);
        return false;
    }
    sensorRegister.scale = std::get<double>(scaleIter->second);

    auto signedIter = configMap.find("Signed");
    if (signedIter == configMap.end())
    {
        error("Missing Signed for {NAME}", "NAME", sensorRegister.name);
        return false;
    }
    sensorRegister.isSigned = std::get<bool>(signedIter->second);
    if (!processRegisterFormat(sensorRegister, configMap))
    {
        return false;
    }
    config.sensorRegisters.emplace_back(sensorRegister);
    return true;
}

static const auto statusBitTypes =
    std::unordered_map<std::string_view, StatusType>{
        {"ControllerFailure", StatusType::controllerFailure},
        {"FanFailure", StatusType::fanFailure},
        {"FilterFailure", StatusType::filterFailure},
        {"PowerFault", StatusType::powerFault},
        {"PumpFailure", StatusType::pumpFailure},
        {"LeakDetectedCritical", StatusType::leakDetectedCritical},
        {"LeakDetectedWarning", StatusType::leakDetectedWarning},
        {"SensorFailure", StatusType::sensorFailure},
        {"SensorReadingCritical", StatusType::sensorReadingCritical},
        {"SensorReadingWarning", StatusType::sensorReadingWarning}};

static auto processStatusBitsInterface(Config& config,
                                       const InventoryBaseConfigMap& configMap)
    -> bool
{
    info("Processing StatusBits for {NAME}", "NAME", config.name);

    StatusBit statusBit = {};

    auto nameIter = configMap.find("Name");
    if (nameIter == configMap.end())
    {
        error("Missing Name for {NAME}", "NAME", config.name);
        return false;
    }
    statusBit.name = std::get<std::string>(nameIter->second);

    auto typeIter = configMap.find("StatusType");
    if (typeIter == configMap.end())
    {
        error("Missing StatusType for {NAME}", "NAME", statusBit.name);
        return false;
    }
    auto type = statusBitTypes.find(std::get<std::string>(typeIter->second));
    if (type == statusBitTypes.end())
    {
        error("Invalid StatusType for {NAME}", "NAME", statusBit.name);
        return false;
    }
    statusBit.type = type->second;

    auto postionIter = configMap.find("Position");
    if (postionIter == configMap.end())
    {
        error("Missing BitPosition for {NAME}", "NAME", statusBit.name);
        return false;
    }
    statusBit.bitPostion = std::get<uint64_t>(postionIter->second);

    auto valueIter = configMap.find("Value");
    if (valueIter == configMap.end())
    {
        error("Missing Value for {NAME}", "NAME", statusBit.name);
        return false;
    }
    statusBit.value = std::get<bool>(valueIter->second);

    auto addressIter = configMap.find("Address");
    if (addressIter == configMap.end())
    {
        error("Missing Address for {NAME}", "NAME", statusBit.name);
        return false;
    }
    auto address = std::get<uint64_t>(addressIter->second);

    config.statusRegisters[address].emplace_back(statusBit);

    return true;
}

static const auto firmwareRegisterTypes =
    std::unordered_map<std::string_view, FirmwareRegisterType>{
        {"Version", FirmwareRegisterType::version},
        {"Update", FirmwareRegisterType::update}};

static auto processFirmwareRegistersInterface(
    Config& config, const InventoryBaseConfigMap& configMap) -> bool
{
    info("Processing FirmwareRegisters for {NAME}", "NAME", config.name);

    FirmwareRegister firmwareRegister = {};

    auto nameIter = configMap.find("Name");
    if (nameIter == configMap.end())
    {
        error("Missing Name for {NAME}", "NAME", config.name);
        return false;
    }
    firmwareRegister.name = std::get<std::string>(nameIter->second);

    auto addressIter = configMap.find("Address");
    if (addressIter == configMap.end())
    {
        error("Missing Address for {NAME}", "NAME", firmwareRegister.name);
        return false;
    }
    firmwareRegister.offset = std::get<uint64_t>(addressIter->second);

    auto sizeIter = configMap.find("Size");
    if (sizeIter == configMap.end())
    {
        error("Missing Size for {NAME}", "NAME", firmwareRegister.name);
        return false;
    }
    firmwareRegister.size = std::get<uint64_t>(sizeIter->second);

    auto typeIter = configMap.find("RegisterType");
    if (typeIter == configMap.end())
    {
        error("Missing RegisterType for {NAME}", "NAME", firmwareRegister.name);
        return false;
    }
    auto type =
        firmwareRegisterTypes.find(std::get<std::string>(typeIter->second));
    if (type == firmwareRegisterTypes.end())
    {
        error("Invalid RegisterType for {NAME}", "NAME", firmwareRegister.name);
        return false;
    }
    firmwareRegister.type = type->second;

    config.firmwareRegisters.emplace_back(firmwareRegister);

    return true;
}

static auto printConfig(const Config& config) -> void
{
    info("Device Config for {NAME}: {ADDRESS} {PORT} {TYPE} {MODEL} {INV_PATH}",
         "NAME", config.name, "ADDRESS", config.address, "PORT",
         config.portName, "TYPE", config.deviceType, "MODEL",
         config.deviceModel, "INV_PATH", config.inventoryPath);

    for (const auto& sensorRegister : config.sensorRegisters)
    {
        info(
            "Sensor Register {NAME} {ADDRESS} {SIZE} {PRECISION} {SCALE} {SIGNED} {FORMAT} {UNIT} {PATH_SUFFIX}",
            "NAME", sensorRegister.name, "ADDRESS", sensorRegister.offset,
            "SIZE", sensorRegister.size, "PRECISION", sensorRegister.precision,
            "SCALE", sensorRegister.scale, "SIGNED", sensorRegister.isSigned,
            "FORMAT", sensorRegister.format, "UNIT", sensorRegister.unit,
            "PATH_SUFFIX", sensorRegister.pathSuffix);
    }

    for (const auto& [address, statusBits] : config.statusRegisters)
    {
        for (const auto& statusBit : statusBits)
        {
            info("Status Bit {NAME} {ADDRESS} {BIT_POSITION} {VALUE} {TYPE}",
                 "NAME", statusBit.name, "ADDRESS", address, "BIT_POSITION",
                 statusBit.bitPostion, "VALUE", statusBit.value, "TYPE",
                 statusBit.type);
        }
    }

    for (const auto& firmwareRegister : config.firmwareRegisters)
    {
        info("Firmware Register {NAME} {ADDRESS} {SIZE} {TYPE}", "NAME",
             firmwareRegister.name, "ADDRESS", firmwareRegister.offset, "SIZE",
             firmwareRegister.size, "TYPE", firmwareRegister.type);
    }
}

static auto getConfigSubTree(
    Config& config, const sdbusplus::message::object_path& path,
    const std::string& interfaceName, const InventoryData& deviceConfig) -> bool
{
    std::string firmwareRegistersInterface =
        interfaceName + ".FirmwareRegisters";
    std::string sensorRegistersInterface = interfaceName + ".SensorRegisters";
    std::string statusBitsInterface = interfaceName + ".StatusBits";

    for (const auto& [curInterface, interfaceConfig] : deviceConfig)
    {
        auto deviceType = validDevices.find(curInterface);
        if (deviceType != validDevices.end())
        {
            if (!processDeviceInterface(config, interfaceConfig, curInterface))
            {
                error("Failed to process {INTERFACE} on {PATH}", "INTERFACE",
                      curInterface, "PATH", path.str);
                return false;
            }
        }
        else if (curInterface.starts_with(sensorRegistersInterface))
        {
            if (!processSensorRegistersInterface(config, interfaceConfig))
            {
                error("Failed to process {INTERFACE} on {PATH}", "INTERFACE",
                      curInterface, "PATH", path.str);
                return false;
            }
        }
        else if (curInterface.starts_with(statusBitsInterface))
        {
            if (!processStatusBitsInterface(config, interfaceConfig))
            {
                error("Failed to process {INTERFACE} on {PATH}", "INTERFACE",
                      curInterface, "PATH", path.str);
                return false;
            }
        }
        else if (curInterface.starts_with(firmwareRegistersInterface))
        {
            if (!processFirmwareRegistersInterface(config, interfaceConfig))
            {
                error("Failed to process {INTERFACE} on {PATH}", "INTERFACE",
                      curInterface, "PATH", path.str);
                return false;
            }
        }
    }

    return true;
}

auto getConfig(sdbusplus::async::context& ctx,
               const sdbusplus::message::object_path& objectPath,
               const std::string& interfaceName)
    -> sdbusplus::async::task<std::optional<Config>>
{
    Config config = {};

    config.inventoryPath = objectPath.parent_path();

    using InventoryIntf =
        sdbusplus::client::xyz::openbmc_project::inventory::Item<>;

    constexpr auto entityManager =
        sdbusplus::async::proxy()
            .service(entity_manager::EntityManagerInterface::serviceName)
            .path(InventoryIntf::namespace_path)
            .interface("org.freedesktop.DBus.ObjectManager");

    for (const auto& [path, deviceConfig] :
         co_await entityManager.call<ManagedObjectType>(ctx,
                                                        "GetManagedObjects"))
    {
        if (path.str != objectPath.str)
        {
            debug("Skipping device {PATH}", "PATH", path.str);
            continue;
        }
        debug("Processing device {PATH}", "PATH", path.str);

        if (!getConfigSubTree(config, path, interfaceName, deviceConfig))
        {
            error("Failed to process device {PATH}", "PATH", path.str);
            co_return std::nullopt;
        }
    }

    printConfig(config);

    co_return config;
}

} // namespace config

Device::Device(sdbusplus::async::context& ctx, const config::Config& config,
               const std::unique_ptr<PortIntf>& serialPort,
               EventIntf::Events& events) :
    ctx(ctx), config(config), serialPort(serialPort), events(events)
{
    ctx.spawn(createSensors());
}

static auto getObjectPath(const std::string& pathSuffix)
    -> sdbusplus::message::object_path
{
    return (sdbusplus::message::object_path(
                SensorValueIntf::namespace_path::value) /
            pathSuffix);
}

auto Device::createSensors() -> sdbusplus::async::task<void>
{
    for (const auto& sensorRegister : config.sensorRegisters)
    {
        SensorValueIntf::properties_t initProperties = {
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(), sensorRegister.unit};

        auto pathSuffix = sensorRegister.pathSuffix + "/" + sensorRegister.name;
        auto sensorPath = getObjectPath(pathSuffix).str.c_str();

        auto sensor =
            std::make_unique<SensorValueIntf>(ctx, sensorPath, initProperties);

        sensor->emit_added();

        sensors.emplace(sensorRegister.name, std::move(sensor));
    }

    ctx.spawn(readSensorRegisters());

    co_return;
}

static auto getRawIntegerFromRegister(const std::vector<uint16_t>& reg,
                                      bool sign) -> uint64_t
{
    std::vector<uint16_t> reg_padded(4, 0);
    size_t copy_size = std::min(reg.size(), reg_padded.size());
    std::copy(reg.begin(), reg.begin() + copy_size, reg_padded.begin());

    if (sign)
    {
        int64_t temp_value =
            std::accumulate(reg_padded.begin(), reg_padded.end(), 0LL,
                            [](int64_t accumulator, uint16_t current_val) {
                                return (accumulator << 16) | current_val;
                            });
        return static_cast<uint64_t>(temp_value);
    }
    else
    {
        return std::accumulate(reg_padded.begin(), reg_padded.end(), 0ULL,
                               [](uint64_t accumulator, uint16_t current_val) {
                                   return (accumulator << 16) | current_val;
                               });
    }
}

auto Device::readSensorRegisters() -> sdbusplus::async::task<void>
{
    while (!ctx.stop_requested())
    {
        for (const auto& sensorRegister : config.sensorRegisters)
        {
            auto sensor = sensors.find(sensorRegister.name);
            if (sensor == sensors.end())
            {
                error("Sensor not found for {NAME}", "NAME",
                      sensorRegister.name);
                continue;
            }

            if (sensorRegister.size > 4)
            {
                error("Unsupported size for register {NAME}", "NAME",
                      sensorRegister.name);
                continue;
            }

            auto registers = std::vector<uint16_t>(sensorRegister.size);
            auto ret = co_await serialPort->readHoldingRegisters(
                config.address, sensorRegister.offset, config.baudRate,
                config.parity, registers);
            if (!ret)
            {
                error(
                    "Failed to read holding registers {NAME} for {DEVICE_ADDRESS}",
                    "NAME", sensorRegister.name, "DEVICE_ADDRESS",
                    config.address);
                continue;
            }

            double regVal = static_cast<double>(
                getRawIntegerFromRegister(registers, sensorRegister.isSigned));
            if (sensorRegister.format == config::SensorFormat::floatingPoint)
            {
                regVal = sensorRegister.shift +
                         (sensorRegister.scale *
                          (regVal / (1ULL << sensorRegister.precision)));
            }

            sensor->second->value(regVal);
        }

        co_await readStatusRegisters();
    }

    co_return;
}

static auto getObjectPath(const config::Config& config, config::StatusType type,
                          const std::string& name)
    -> sdbusplus::message::object_path
{
    switch (type)
    {
        case config::StatusType::sensorReadingCritical:
        case config::StatusType::sensorReadingWarning:
        case config::StatusType::sensorFailure:
            return sdbusplus::message::object_path(
                std::string(SensorValueIntf::namespace_path::value) + "/" +
                name);
        case config::StatusType::controllerFailure:
            return config.inventoryPath;
        case config::StatusType::pumpFailure:
            return sdbusplus::message::object_path(
                "/xyz/openbmc_project/state/pump/" + name);
        case config::StatusType::filterFailure:
            return sdbusplus::message::object_path(
                "/xyz/openbmc_project/state/filter/" + name);
        case config::StatusType::powerFault:
            return sdbusplus::message::object_path(
                "/xyz/openbmc_project/state/power_rail/" + name);
        case config::StatusType::fanFailure:
            return sdbusplus::message::object_path(
                "/xyz/openbmc_project/state/fan/" + name);
        case config::StatusType::leakDetectedCritical:
        case config::StatusType::leakDetectedWarning:
            using DetectorIntf =
                sdbusplus::aserver::xyz::openbmc_project::state::leak::Detector<
                    Device>;
            return sdbusplus::message::object_path(
                std::string(DetectorIntf::namespace_path::value) + "/" +
                DetectorIntf::namespace_path::detector + "/" + name);
        case config::StatusType::unknown:
            error("Unknown status type for {NAME}", "NAME", name);
    }

    return sdbusplus::message::object_path();
}

auto Device::readStatusRegisters() -> sdbusplus::async::task<void>
{
    for (const auto& [address, statusBits] : config.statusRegisters)
    {
        auto registers = std::vector<uint16_t>(1);
        auto ret = co_await serialPort->readHoldingRegisters(
            config.address, address, config.baudRate, config.parity, registers);
        if (!ret)
        {
            error("Failed to read holding registers for {DEVICE_ADDRESS}",
                  "DEVICE_ADDRESS", config.address);
            continue;
        }

        for (const auto& statusBit : statusBits)
        {
            auto statusBitValue = (registers[0] & (1 << statusBit.bitPostion));
            auto statusAsserted = (statusBitValue == statusBit.value);
            auto objectPath =
                getObjectPath(config, statusBit.type, statusBit.name);
            double sensorValue = std::numeric_limits<double>::quiet_NaN();
            SensorValueIntf::Unit sensorUnit = SensorValueIntf::Unit::Percent;
            auto sensorIter = sensors.find(statusBit.name);
            if (sensorIter != sensors.end())
            {
                sensorValue = sensorIter->second->value();
                sensorUnit = sensorIter->second->unit();
            }

            co_await generateEvent(statusBit, objectPath, sensorValue,
                                   sensorUnit, statusAsserted);
        }
    }

    co_return;
}

auto Device::generateEvent(const config::StatusBit& statusBit,
                           const sdbusplus::message::object_path& objectPath,
                           double sensorValue, SensorValueIntf::Unit sensorUnit,
                           bool statusAsserted) -> sdbusplus::async::task<void>
{
    switch (statusBit.type)
    {
        case config::StatusType::sensorReadingCritical:
            co_await events.generateSensorReadingEvent(
                objectPath, EventIntf::EventLevel::critical, sensorValue,
                sensorUnit, statusAsserted);
            break;
        case config::StatusType::sensorReadingWarning:
            co_await events.generateSensorReadingEvent(
                objectPath, EventIntf::EventLevel::warning, sensorValue,
                sensorUnit, statusAsserted);
            break;
        case config::StatusType::sensorFailure:
            co_await events.generateSensorFailureEvent(objectPath,
                                                       statusAsserted);
            break;
        case config::StatusType::controllerFailure:
            co_await events.generateControllerFailureEvent(
                objectPath, statusBit.name, statusAsserted);
            break;
        case config::StatusType::powerFault:
            co_await events.generatePowerFaultEvent(objectPath, statusBit.name,
                                                    statusAsserted);
            break;
        case config::StatusType::filterFailure:
            co_await events.generateFilterFailureEvent(objectPath,
                                                       statusAsserted);
            break;
        case config::StatusType::pumpFailure:
            co_await events.generatePumpFailureEvent(objectPath,
                                                     statusAsserted);
            break;
        case config::StatusType::fanFailure:
            co_await events.generateFanFailureEvent(objectPath, statusAsserted);
            break;
        case config::StatusType::leakDetectedCritical:
            co_await events.generateLeakDetectedEvent(
                objectPath, EventIntf::EventLevel::critical, statusAsserted);
            break;
        case config::StatusType::leakDetectedWarning:
            co_await events.generateLeakDetectedEvent(
                objectPath, EventIntf::EventLevel::warning, statusAsserted);
            break;
        case config::StatusType::unknown:
            error("Unknown status type for {NAME}", "NAME", statusBit.name);
            break;
    }
}

} // namespace phosphor::modbus::rtu::device
