#include "base_config.hpp"

#include "common/entity_manager_interface.hpp"

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Inventory/Item/client.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/aserver.hpp>

#include <flat_map>

namespace phosphor::modbus::rtu::device::config
{

PHOSPHOR_LOG2_USING;

using BasicVariantType =
    std::variant<std::vector<std::string>, std::vector<uint8_t>, std::string,
                 int64_t, uint64_t, double, int32_t, uint32_t, int16_t,
                 uint16_t, uint8_t, bool>;
using InventoryBaseConfigMap = std::flat_map<std::string, BasicVariantType>;
using InventoryData = std::flat_map<std::string, InventoryBaseConfigMap>;
using ManagedObjectType =
    std::flat_map<sdbusplus::message::object_path, InventoryData>;

static constexpr std::array<std::pair<std::string_view, Parity>, 3>
    validParities = {
        {{"Odd", Parity::odd}, {"Even", Parity::even}, {"None", Parity::none}}};

template <typename T>
auto getValue(const InventoryBaseConfigMap& configMap, const std::string& key,
              const std::string& contextName) -> T
{
    auto iter = configMap.find(key);
    if (iter == configMap.end())
    {
        throw std::runtime_error(
            "Missing property " + key + " for " + contextName);
    }

    try
    {
        return std::get<T>(iter->second);
    }
    catch (const std::bad_variant_access& ex)
    {
        throw std::runtime_error(
            "Incorrect type for property " + key + " in " + contextName);
    }
}

static inline auto getDataParity(Config& config,
                                 const InventoryBaseConfigMap& configMap)
    -> void
{
    auto receivedParity =
        getValue<std::string>(configMap, "DataParity", config.name);

    for (const auto& [parityStr, parity] : validParities)
    {
        if (parityStr == receivedParity)
        {
            config.parity = parity;
            break;
        }
    }

    if (config.parity == Parity::unknown)
    {
        throw std::runtime_error(
            "Invalid parity " + receivedParity + " for " + config.name);
    }
}

static auto processDeviceInterface(Config& config,
                                   const InventoryBaseConfigMap& configMap)
    -> void
{
    debug("Processing device config");

    config.name = getValue<std::string>(configMap, "Name", config.name);

    std::replace(config.name.begin(), config.name.end(), ' ', '_');

    config.address = getValue<uint64_t>(configMap, "Address", config.name);

    getDataParity(config, configMap);

    config.baudRate = getValue<uint64_t>(configMap, "BaudRate", config.name);

    config.portName =
        getValue<std::string>(configMap, "SerialPort", config.name);

    getValue<std::string>(configMap, "Type", config.name);
}

static const auto sensorTypes = std::unordered_map<
    std::string_view, std::pair<std::string_view, SensorValueIntf::Unit>>{
    {"FanTach",
     {SensorValueIntf::namespace_path::fan_tach, SensorValueIntf::Unit::RPMS}},
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
};

static const auto formatTypes =
    std::unordered_map<std::string_view, SensorFormat>{
        {"Integer", SensorFormat::integer},
        {"Float", SensorFormat::floatingPoint}};

static auto processRegisterType(SensorRegister& sensorRegister,
                                const InventoryBaseConfigMap& configMap) -> void
{
    auto registerType =
        getValue<std::string>(configMap, "RegisterType", sensorRegister.name);

    auto type = sensorTypes.find(registerType);
    if (type == sensorTypes.end())
    {
        throw std::runtime_error("Invalid RegisterType " + registerType +
                                 " for " + sensorRegister.name);
    }
    sensorRegister.pathSuffix = type->second.first;
    sensorRegister.unit = type->second.second;
}

static auto processRegisterFormat(SensorRegister& sensorRegister,
                                  const InventoryBaseConfigMap& configMap)
    -> void
{
    auto format =
        getValue<std::string>(configMap, "Format", sensorRegister.name);

    auto formatIter = formatTypes.find(format);
    if (formatIter == formatTypes.end())
    {
        throw std::runtime_error(
            "Invalid Format " + format + " for " + sensorRegister.name);
    }
    sensorRegister.format = formatIter->second;
}

static auto processSensorRegistersInterface(
    Config& config, const InventoryBaseConfigMap& configMap) -> void
{
    SensorRegister sensorRegister = {};

    sensorRegister.name = getValue<std::string>(configMap, "Name", config.name);

    processRegisterType(sensorRegister, configMap);

    sensorRegister.offset =
        getValue<uint64_t>(configMap, "Address", config.name);

    sensorRegister.size = getValue<uint64_t>(configMap, "Size", config.name);

    sensorRegister.precision =
        getValue<uint64_t>(configMap, "Precision", config.name);

    sensorRegister.shift = getValue<double>(configMap, "Shift", config.name);

    sensorRegister.scale = getValue<double>(configMap, "Scale", config.name);

    sensorRegister.isSigned = getValue<bool>(configMap, "Signed", config.name);

    processRegisterFormat(sensorRegister, configMap);

    config.sensorRegisters.emplace_back(sensorRegister);
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
    -> void
{
    debug("Processing StatusBits for {NAME}", "NAME", config.name);

    StatusBit statusBit = {};

    statusBit.name = getValue<std::string>(configMap, "Name", config.name);

    auto type = getValue<std::string>(configMap, "StatusType", config.name);
    auto typeIter = statusBitTypes.find(type);
    if (typeIter == statusBitTypes.end())
    {
        throw std::runtime_error(
            "Invalid StatusType " + type + " for " + statusBit.name);
    }
    statusBit.type = typeIter->second;

    statusBit.bitPosition =
        getValue<uint64_t>(configMap, "BitPosition", config.name);

    statusBit.value = getValue<bool>(configMap, "Value", config.name);

    auto address = getValue<uint64_t>(configMap, "Address", config.name);

    config.statusRegisters[address].emplace_back(statusBit);
}

static const auto firmwareRegisterTypes =
    std::unordered_map<std::string_view, FirmwareRegisterType>{
        {"Version", FirmwareRegisterType::version},
        {"Update", FirmwareRegisterType::update}};

static auto processFirmwareRegistersInterface(
    Config& config, const InventoryBaseConfigMap& configMap) -> void
{
    debug("Processing FirmwareRegisters for {NAME}", "NAME", config.name);

    FirmwareRegister firmwareRegister = {};

    firmwareRegister.name =
        getValue<std::string>(configMap, "Name", config.name);

    firmwareRegister.offset =
        getValue<uint64_t>(configMap, "Address", firmwareRegister.name);

    firmwareRegister.size =
        getValue<uint64_t>(configMap, "Size", firmwareRegister.name);

    auto registerType =
        getValue<std::string>(configMap, "RegisterType", firmwareRegister.name);
    auto registerTypeIter = firmwareRegisterTypes.find(registerType);
    if (registerTypeIter == firmwareRegisterTypes.end())
    {
        throw std::runtime_error("Invalid RegisterType " + registerType +
                                 " for " + firmwareRegister.name);
    }
    firmwareRegister.type = registerTypeIter->second;

    config.firmwareRegisters.emplace_back(firmwareRegister);
}

static auto printConfig(const Config& config) -> void
{
    info("Device Config for {NAME}: {ADDRESS} {PORT} {INV_PATH}", "NAME",
         config.name, "ADDRESS", config.address, "PORT", config.portName,
         "INV_PATH", config.inventoryPath);

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
                 statusBit.bitPosition, "VALUE", statusBit.value, "TYPE",
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

static auto getConfigSubTree(Config& config, const std::string& interfaceName,
                             const InventoryData& deviceConfig) -> void
{
    std::string firmwareRegistersInterface =
        interfaceName + ".FirmwareRegisters";
    std::string sensorRegistersInterface = interfaceName + ".SensorRegisters";
    std::string statusBitsInterface = interfaceName + ".StatusBits";

    for (const auto& [curInterface, interfaceConfig] : deviceConfig)
    {
        if (curInterface == interfaceName)
        {
            processDeviceInterface(config, interfaceConfig);
        }
        else if (curInterface.starts_with(sensorRegistersInterface))
        {
            processSensorRegistersInterface(config, interfaceConfig);
        }
        else if (curInterface.starts_with(statusBitsInterface))
        {
            processStatusBitsInterface(config, interfaceConfig);
        }
        else if (curInterface.starts_with(firmwareRegistersInterface))
        {
            processFirmwareRegistersInterface(config, interfaceConfig);
        }
    }
}

auto updateBaseConfig(sdbusplus::async::context& ctx,
                      const sdbusplus::message::object_path& objectPath,
                      const std::string& interfaceName, Config& config)
    -> sdbusplus::async::task<bool>
{
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

        try
        {
            getConfigSubTree(config, interfaceName, deviceConfig);
        }
        catch (std::exception& e)
        {
            error("Failed to process device {PATH} with {ERROR}", "PATH",
                  path.str, "ERROR", e);
            co_return false;
        }
    }

    printConfig(config);

    co_return true;
}

} // namespace phosphor::modbus::rtu::device::config
