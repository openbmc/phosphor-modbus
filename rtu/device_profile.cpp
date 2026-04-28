#include "device_profile.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace phosphor::modbus::rtu::profile
{

using json = nlohmann::json;

static constexpr auto profileDir = PROFILE_DIR;

static const std::unordered_map<std::string, Parity> parityMap = {
    {"None", Parity::none},
    {"Even", Parity::even},
    {"Odd", Parity::odd},
};

static const std::unordered_map<std::string, InventoryDataType>
    inventoryDataTypeMap = {
        {"BuildDate", InventoryDataType::buildDate},
        {"Manufacturer", InventoryDataType::manufacturer},
        {"Model", InventoryDataType::model},
        {"PartNumber", InventoryDataType::partNumber},
        {"SerialNumber", InventoryDataType::serialNumber},
        {"SparePartNumber", InventoryDataType::sparePartNumber},
};

static const std::unordered_map<std::string, SensorType> sensorTypeMap = {
    {"FanTach", SensorType::fanTach},
    {"LiquidFlow", SensorType::liquidFlow},
    {"Power", SensorType::power},
    {"Pressure", SensorType::pressure},
    {"Temperature", SensorType::temperature},
    {"Voltage", SensorType::voltage},
    {"Current", SensorType::current},
    {"Airflow", SensorType::airflow},
    {"Altitude", SensorType::altitude},
    {"Energy", SensorType::energy},
    {"Frequency", SensorType::frequency},
    {"Humidity", SensorType::humidity},
    {"Utilization", SensorType::utilization},
    {"Valve", SensorType::valve},
};

static const std::unordered_map<std::string, SensorFormat> sensorFormatMap = {
    {"FloatingPoint", SensorFormat::floatingPoint},
    {"Integer", SensorFormat::integer},
};

static const std::unordered_map<std::string, StatusType> statusTypeMap = {
    {"ControllerFailure", StatusType::controllerFailure},
    {"FanFailure", StatusType::fanFailure},
    {"FilterFailure", StatusType::filterFailure},
    {"PowerFault", StatusType::powerFault},
    {"PumpFailure", StatusType::pumpFailure},
    {"LeakDetectedCritical", StatusType::leakDetectedCritical},
    {"LeakDetectedWarning", StatusType::leakDetectedWarning},
    {"SensorFailure", StatusType::sensorFailure},
    {"SensorReadingCritical", StatusType::sensorReadingCritical},
    {"SensorReadingWarning", StatusType::sensorReadingWarning},
};

static const std::unordered_map<std::string, FirmwareRegisterType>
    firmwareRegisterTypeMap = {
        {"Version", FirmwareRegisterType::version},
        {"Update", FirmwareRegisterType::update},
};

static const std::unordered_map<std::string, DeviceType> deviceTypeMap = {
    {"BatteryBackupUnit", DeviceType::batteryBackupUnit},
    {"CapacitorBankUnit", DeviceType::capacitorBankUnit},
    {"FlowMeter", DeviceType::flowMeter},
    {"HeatExchanger", DeviceType::heatExchanger},
    {"PowerMonitorModule", DeviceType::powerMonitorModule},
    {"PowerSupplyUnit", DeviceType::powerSupplyUnit},
    {"ReservoirPumpUnit", DeviceType::reservoirPumpUnit},
    {"Valve", DeviceType::valve},
};

static const std::unordered_map<std::string, DeviceModel> deviceModelMap = {
    {"Artesyn7000433970000", DeviceModel::Artesyn7000433970000},
    {"Danfoss003Z8540", DeviceModel::Danfoss003Z8540},
    {"DeltaECD70000020", DeviceModel::DeltaECD70000020},
    {"DeltaRDF040DSS5193E0", DeviceModel::DeltaRDF040DSS5193E0},
    {"PanasonicBJBPM102A0001", DeviceModel::PanasonicBJBPM102A0001},
};

template <typename EnumType>
static auto lookupEnum(const std::unordered_map<std::string, EnumType>& map,
                       const std::string& value, const std::string& fieldName)
    -> EnumType
{
    auto it = map.find(value);
    if (it == map.end())
    {
        throw std::invalid_argument("Unknown " + fieldName + ": " + value);
    }
    return it->second;
}

static auto parseInventoryRegisters(const json& j)
    -> std::vector<InventoryRegister>
{
    std::vector<InventoryRegister> registers;
    std::unordered_set<std::string> names;
    for (const auto& reg : j)
    {
        auto name = reg.at("Name").get<std::string>();
        if (!names.insert(name).second)
        {
            throw std::invalid_argument(
                "Duplicate inventory register name: " + name);
        }
        registers.push_back(
            {.name = name,
             .type = lookupEnum(inventoryDataTypeMap,
                                reg.at("Type").get<std::string>(), "Type"),
             .offset = reg.at("Offset").get<uint16_t>(),
             .size = reg.at("Size").get<uint8_t>()});
    }
    return registers;
}

static auto parseSensorRegisters(const json& j) -> std::vector<SensorRegister>
{
    std::vector<SensorRegister> registers;
    std::unordered_set<std::string> names;
    for (const auto& reg : j)
    {
        auto name = reg.at("Name").get<std::string>();
        if (!names.insert(name).second)
        {
            throw std::invalid_argument(
                "Duplicate sensor register name: " + name);
        }
        registers.push_back(
            {.name = name,
             .type = lookupEnum(sensorTypeMap,
                                reg.at("Type").get<std::string>(), "Type"),
             .offset = reg.at("Offset").get<uint16_t>(),
             .size = reg.at("Size").get<uint8_t>(),
             .precision = reg.value("Precision", static_cast<uint8_t>(0)),
             .scale = reg.value("Scale", 1.0),
             .shift = reg.value("Shift", 0.0),
             .isSigned = reg.value("IsSigned", false),
             .format =
                 lookupEnum(sensorFormatMap,
                            reg.at("Format").get<std::string>(), "Format")});
    }
    return registers;
}

static auto parseStatusRegisters(const json& j)
    -> std::unordered_map<uint16_t, std::vector<StatusBit>>
{
    std::unordered_map<uint16_t, std::vector<StatusBit>> statusRegisters;
    for (const auto& reg : j)
    {
        auto address = reg.at("Offset").get<uint16_t>();
        std::vector<StatusBit> bits;
        std::unordered_set<std::string> names;
        for (const auto& bit : reg.at("Bits"))
        {
            auto name = bit.at("Name").get<std::string>();
            if (!names.insert(name).second)
            {
                throw std::invalid_argument(
                    "Duplicate status bit name: " + name);
            }
            bits.push_back(
                {.name = name,
                 .type = lookupEnum(statusTypeMap,
                                    bit.at("Type").get<std::string>(), "Type"),
                 .bitPosition = bit.at("BitPosition").get<uint8_t>(),
                 .value = bit.at("Value").get<bool>()});
        }
        statusRegisters[address] = std::move(bits);
    }
    return statusRegisters;
}

static auto parseFirmwareRegisters(const json& j)
    -> std::vector<FirmwareRegister>
{
    std::vector<FirmwareRegister> registers;
    std::unordered_set<std::string> names;
    for (const auto& reg : j)
    {
        auto name = reg.at("Name").get<std::string>();
        if (!names.insert(name).second)
        {
            throw std::invalid_argument(
                "Duplicate firmware register name: " + name);
        }
        registers.push_back(
            {.name = name,
             .type = lookupEnum(firmwareRegisterTypeMap,
                                reg.at("Type").get<std::string>(), "Type"),
             .offset = reg.at("Offset").get<uint16_t>(),
             .size = reg.at("Size").get<uint8_t>()});
    }
    return registers;
}

struct DeviceProfileEntry
{
    DeviceType deviceType;
    DeviceModel deviceModel;
    DeviceProfile profile;
};

static auto parseProfileEntry(const std::filesystem::path& path)
    -> DeviceProfileEntry
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open profile: " + path.string());
    }

    auto j = json::parse(file);

    DeviceProfileEntry entry;
    entry.deviceType = lookupEnum(
        deviceTypeMap, j.at("DeviceType").get<std::string>(), "DeviceType");
    entry.deviceModel = lookupEnum(
        deviceModelMap, j.at("DeviceModel").get<std::string>(), "DeviceModel");

    const auto& probe = j.at("ProbeRegister");
    entry.profile.probeRegister.offset = probe.at("Offset").get<uint16_t>();
    entry.profile.probeRegister.size = probe.at("Size").get<uint8_t>();
    const auto& expectedValue = probe.at("ExpectedValue");
    if (expectedValue.is_string())
    {
        entry.profile.probeRegister.expectedValue =
            expectedValue.get<std::string>();
    }
    else
    {
        entry.profile.probeRegister.expectedValue =
            expectedValue.get<uint64_t>();
    }

    entry.profile.parity =
        lookupEnum(parityMap, j.at("Parity").get<std::string>(), "Parity");
    entry.profile.baudRate = j.at("BaudRate").get<uint32_t>();

    if (j.contains("InventoryRegisters"))
    {
        entry.profile.inventoryRegisters =
            parseInventoryRegisters(j["InventoryRegisters"]);
    }
    if (j.contains("SensorRegisters"))
    {
        entry.profile.sensorRegisters =
            parseSensorRegisters(j["SensorRegisters"]);
    }
    if (j.contains("StatusRegisters"))
    {
        entry.profile.statusRegisters =
            parseStatusRegisters(j["StatusRegisters"]);
    }
    if (j.contains("FirmwareRegisters"))
    {
        entry.profile.firmwareRegisters =
            parseFirmwareRegisters(j["FirmwareRegisters"]);
    }

    return entry;
}

static std::unordered_map<std::string, DeviceProfileEntry> deviceProfiles;

static auto findEntry(std::string_view type) -> const DeviceProfileEntry&
{
    auto key = std::string(type);
    auto it = deviceProfiles.find(key);
    if (it != deviceProfiles.end())
    {
        return it->second;
    }

    auto path = std::filesystem::path(profileDir) / (key + ".json");
    auto [inserted,
          success] = deviceProfiles.emplace(key, parseProfileEntry(path));
    if (!success)
    {
        throw std::runtime_error("Failed to cache profile: " + key);
    }
    return inserted->second;
}

auto getDeviceProfile(std::string_view type) -> const DeviceProfile&
{
    return findEntry(type).profile;
}

auto getDeviceType(std::string_view type) -> DeviceType
{
    return findEntry(type).deviceType;
}

auto getDeviceModel(std::string_view type) -> DeviceModel
{
    return findEntry(type).deviceModel;
}

auto getProfileNames() -> std::vector<std::string>
{
    std::vector<std::string> names;
    for (const auto& entry : std::filesystem::directory_iterator(profileDir))
    {
        if (entry.path().extension() == ".json")
        {
            names.emplace_back(entry.path().stem().string());
        }
    }
    return names;
}

} // namespace phosphor::modbus::rtu::profile
