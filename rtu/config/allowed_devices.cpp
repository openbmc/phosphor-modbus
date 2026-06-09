#include "allowed_devices.hpp"

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
static constexpr auto configFileName = "allowed-devices.json";

AllowedDevices::AllowedDevices(sdbusplus::async::context& ctx) :
    ctx(ctx),
    notifyWatch(ctx, configDir,
                std::bind_front(&AllowedDevices::configUpdateHandler, this))
{
    loadConfig();
}

auto AllowedDevices::startWatching() -> void
{
    ctx.spawn(notifyWatch.readNotifyAsync());
}

auto AllowedDevices::configUpdateHandler(std::string fileName)
    -> sdbusplus::async::task<>
{
    if (fileName.find(configFileName) == std::string::npos)
    {
        co_return;
    }

    loadConfig();
}

auto AllowedDevices::loadConfig() -> void
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
        allowlist.emplace();
        for (const auto& entry : j.at("AllowedDevices"))
        {
            auto name = entry.get<std::string>();
            std::replace(name.begin(), name.end(), ' ', '_');
            allowlist->insert(std::move(name));
        }
        info("Loaded {COUNT} allowed devices from config", "COUNT",
             allowlist->size());
    }
    catch (const std::exception& e)
    {
        error("Failed to parse allowed devices config: {ERROR}", "ERROR", e);
        allowlist.reset();
    }
}

auto AllowedDevices::isAllowed(const std::string& deviceName) const -> bool
{
    if (!allowlist)
    {
        return true;
    }
    return allowlist->contains(deviceName);
}

} // namespace phosphor::modbus::rtu::config
