#include "connected_devices.hpp"

#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>

namespace phosphor::modbus::rtu::config
{

PHOSPHOR_LOG2_USING;

using json = nlohmann::json;

static constexpr auto configDir = CONFIG_DIR;
static constexpr auto configFileName = "connected-devices.json";

static void from_json(const json& j, std::unordered_set<std::string>& devices)
{
    for (const auto& entry : j.at("ConnectedDevices"))
    {
        auto name = entry.get<std::string>();
        std::replace(name.begin(), name.end(), ' ', '_');
        devices.insert(std::move(name));
    }
}

ConnectedDevices::ConnectedDevices(sdbusplus::async::context& ctx) :
    ctx(ctx),
    notifyWatch(ctx, configDir,
                std::bind_front(&ConnectedDevices::configUpdateHandler, this))
{
    loadConfig();
}

auto ConnectedDevices::startWatching() -> void
{
    ctx.spawn(notifyWatch.readNotifyAsync());
}

auto ConnectedDevices::configUpdateHandler(std::string fileName)
    -> sdbusplus::async::task<>
{
    if (fileName.find(configFileName) == std::string::npos)
    {
        co_return;
    }

    loadConfig();
}

auto ConnectedDevices::loadConfig() -> void
{
    auto configPath = std::filesystem::path(configDir) / configFileName;

    std::ifstream file(configPath);
    if (!file.is_open())
    {
        allowlist.reset();
        return;
    }

    try
    {
        auto j = json::parse(file);
        std::unordered_set<std::string> devices;
        from_json(j, devices);
        allowlist.emplace(std::move(devices));
        info("Loaded {COUNT} connected devices from config", "COUNT",
             allowlist->size());
    }
    catch (const std::exception& e)
    {
        error("Failed to parse connected devices config: {ERROR}", "ERROR", e);
        allowlist.reset();
    }
}

auto ConnectedDevices::isAllowed(const std::string& deviceName) const -> bool
{
    if (!allowlist)
    {
        return true;
    }
    return allowlist->contains(deviceName);
}

} // namespace phosphor::modbus::rtu::config
