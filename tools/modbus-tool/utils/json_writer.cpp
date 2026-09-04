#include "utils/json_writer.hpp"

#include <chrono>
#include <format>
#include <fstream>
#include <string>
#include <string_view>

namespace modbus_tool
{

namespace
{

constexpr auto schemaVersion = "1.0.0";
constexpr auto toolName = "modbus-tool";
constexpr auto osRelease = "/etc/os-release";

auto resultName(Result result) -> std::string_view
{
    switch (result)
    {
        case Result::success:
            return "Success";
        case Result::partial:
            return "Partial";
        case Result::failure:
            return "Failure";
    }
    return "Failure";
}

/** @brief VERSION_ID from os-release, which identifies the image.
 *
 *  The value may or may not be quoted, so strip the quotes either way. */
auto bmcVersion() -> std::string
{
    constexpr std::string_view key = "VERSION_ID=";

    std::ifstream file(osRelease);
    for (std::string line; std::getline(file, line);)
    {
        if (!line.starts_with(key))
        {
            continue;
        }
        auto value = line.substr(key.size());
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
        {
            value = value.substr(1, value.size() - 2);
        }
        return value;
    }

    return "unknown";
}

auto timestamp() -> std::string
{
    return std::format("{:%FT%TZ}", std::chrono::floor<std::chrono::seconds>(
                                        std::chrono::system_clock::now()));
}

auto toJson(const RegisterDump& reg) -> nlohmann::ordered_json
{
    nlohmann::ordered_json out;
    out["Name"] = reg.name;
    out["Offset"] = std::format("0x{:X}", reg.offset);
    out["Size"] = reg.size;
    out["ReadStatus"] = reg.read ? "Success" : "Failure";

    auto raw = nlohmann::ordered_json::array();
    for (auto word : reg.raw)
    {
        raw.emplace_back(std::format("0x{:04X}", word));
    }
    out["Raw"] = std::move(raw);

    if (!reg.bits.empty())
    {
        auto bits = nlohmann::ordered_json::array();
        for (const auto& bit : reg.bits)
        {
            bits.emplace_back(nlohmann::ordered_json{
                {"Name", bit.name},
                {"Position", bit.position},
                {"Type", bit.type},
                {"Asserted", bit.asserted}});
        }
        out["Bits"] = std::move(bits);
    }

    return out;
}

auto toJson(const std::vector<RegisterDump>& registers)
    -> nlohmann::ordered_json
{
    auto out = nlohmann::ordered_json::array();
    for (const auto& reg : registers)
    {
        out.emplace_back(toJson(reg));
    }
    return out;
}

auto toJson(const DeviceDump& device) -> nlohmann::ordered_json
{
    nlohmann::ordered_json out;
    out["Name"] = device.name;
    out["Type"] = device.type;
    out["Address"] = std::format("0x{:X}", device.address);
    out["SerialPort"] = device.serialPort;
    out["Result"] = resultName(device.result);
    if (!device.reason.empty())
    {
        out["Reason"] = device.reason;
    }

    out["Registers"] = {
        {"Inventory", toJson(device.registers.inventory)},
        {"Firmware", toJson(device.registers.firmware)},
        {"Sensor", toJson(device.registers.sensor)},
        {"Status", toJson(device.registers.status)},
        {"Metric", toJson(device.registers.metric)},
        {"Config", toJson(device.registers.config)},
    };

    return out;
}

} // namespace

auto toJson(const Dump& dump) -> nlohmann::ordered_json
{
    auto devices = nlohmann::ordered_json::array();
    for (const auto& device : dump.devices)
    {
        devices.emplace_back(toJson(device));
    }

    return nlohmann::ordered_json{
        {"Metadata",
         {{"SchemaVersion", schemaVersion},
          {"Tool", toolName},
          {"BMCVersion", bmcVersion()},
          {"Timestamp", timestamp()}}},
        {"Devices", std::move(devices)}};
}

} // namespace modbus_tool
