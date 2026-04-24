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
static constexpr auto defaultPrecision = static_cast<uint8_t>(0);
static constexpr auto defaultScale = 1.0;
static constexpr auto defaultShift = 0.0;
static constexpr auto defaultIsSigned = false;

static std::unordered_map<std::string, DeviceProfile> deviceProfiles;

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

static void from_json(const json& j, InventoryRegister& r)
{
    r.name = j.at("Name").get<std::string>();
    r.type = lookupEnum(inventoryDataTypeMap, j.at("Type").get<std::string>(),
                        "Type");
    r.offset = j.at("Offset").get<uint16_t>();
    r.size = j.at("Size").get<uint8_t>();
}

static void from_json(const json& j, SensorRegister& r)
{
    r.name = j.at("Name").get<std::string>();
    r.type = lookupEnum(sensorTypeMap, j.at("Type").get<std::string>(), "Type");
    r.offset = j.at("Offset").get<uint16_t>();
    r.size = j.at("Size").get<uint8_t>();
    r.precision = j.value("Precision", defaultPrecision);
    r.scale = j.value("Scale", defaultScale);
    r.shift = j.value("Shift", defaultShift);
    r.isSigned = j.value("IsSigned", defaultIsSigned);
    r.format = lookupEnum(sensorFormatMap, j.at("Format").get<std::string>(),
                          "Format");
}

static void from_json(const json& j, StatusBit& b)
{
    b.name = j.at("Name").get<std::string>();
    b.type = lookupEnum(statusTypeMap, j.at("Type").get<std::string>(), "Type");
    b.bitPosition = j.at("BitPosition").get<uint8_t>();
    b.value = j.at("Value").get<bool>();
}

template <typename T, typename NameFn>
static void validateUniqueNames(const std::vector<T>& items,
                                const std::string& context, NameFn getName)
{
    std::unordered_set<std::string> names;
    for (const auto& item : items)
    {
        auto name = getName(item);
        if (!names.insert(name).second)
        {
            throw std::invalid_argument("Duplicate " + context + ": " + name);
        }
    }
}

static auto parseStatusRegisters(const json& j)
    -> std::unordered_map<uint16_t, std::vector<StatusBit>>
{
    std::unordered_map<uint16_t, std::vector<StatusBit>> statusRegisters;
    for (const auto& reg : j)
    {
        auto address = reg.at("Offset").get<uint16_t>();
        auto bits = reg.at("Bits").get<std::vector<StatusBit>>();
        validateUniqueNames(bits, "status bit name", [](const StatusBit& b) {
            return b.name;
        });
        statusRegisters[address] = std::move(bits);
    }
    return statusRegisters;
}

static void from_json(const json& j, FirmwareRegister& r)
{
    r.name = j.at("Name").get<std::string>();
    r.type = lookupEnum(firmwareRegisterTypeMap,
                        j.at("Type").get<std::string>(), "Type");
    r.offset = j.at("Offset").get<uint16_t>();
    r.size = j.at("Size").get<uint8_t>();
}

static auto parseProfile(const std::filesystem::path& path) -> DeviceProfile
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open profile: " + path.string());
    }

    auto j = json::parse(file);

    DeviceProfile profile;
    profile.parity =
        lookupEnum(parityMap, j.at("Parity").get<std::string>(), "Parity");
    profile.baudRate = j.at("BaudRate").get<uint32_t>();

    if (j.contains("InventoryRegisters"))
    {
        profile.inventoryRegisters =
            j["InventoryRegisters"].get<std::vector<InventoryRegister>>();
    }
    if (j.contains("SensorRegisters"))
    {
        profile.sensorRegisters =
            j["SensorRegisters"].get<std::vector<SensorRegister>>();
        validateUniqueNames(profile.sensorRegisters, "sensor register name",
                            [](const SensorRegister& r) { return r.name; });
    }
    if (j.contains("StatusRegisters"))
    {
        profile.statusRegisters = parseStatusRegisters(j["StatusRegisters"]);
    }
    if (j.contains("FirmwareRegisters"))
    {
        profile.firmwareRegisters =
            j["FirmwareRegisters"].get<std::vector<FirmwareRegister>>();
    }

    return profile;
}

auto getDeviceProfile(std::string_view type) -> const DeviceProfile&
{
    auto key = std::string(type);
    auto it = deviceProfiles.find(key);
    if (it != deviceProfiles.end())
    {
        return it->second;
    }

    auto path = std::filesystem::path(profileDir) / (key + ".json");
    auto [inserted, success] = deviceProfiles.emplace(key, parseProfile(path));
    if (!success)
    {
        throw std::runtime_error("Failed to cache profile: " + key);
    }
    return inserted->second;
}

} // namespace phosphor::modbus::rtu::profile
