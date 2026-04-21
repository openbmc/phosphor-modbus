#include "device_factory.hpp"

#include "power_monitor_module.hpp"
#include "reservoir_pump_unit.hpp"

#include <phosphor-logging/lg2.hpp>

#include <string>
#include <vector>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

static constexpr auto configInterfacePrefix =
    "xyz.openbmc_project.Configuration.";

auto DeviceFactory::getInterfaces() -> std::vector<std::string>
{
    auto names = ProfileIntf::getProfileNames();
    std::vector<std::string> interfaces;
    interfaces.reserve(names.size());
    for (const auto& name : names)
    {
        interfaces.emplace_back(
            std::string(configInterfacePrefix) + std::string(name));
    }
    return interfaces;
}

auto DeviceFactory::getConfig(sdbusplus::async::context& ctx,
                              const sdbusplus::object_path& objectPath,
                              const std::string& interfaceName)
    -> sdbusplus::async::task<std::optional<config::DeviceFactoryConfig>>
{
    auto baseConfig =
        co_await config::getConfig(ctx, objectPath, interfaceName);
    if (!baseConfig)
    {
        co_return std::nullopt;
    }

    auto profileName =
        interfaceName.substr(std::string_view(configInterfacePrefix).size());

    co_return config::DeviceFactoryConfig{
        {std::move(*baseConfig)},
        ProfileIntf::getDeviceType(profileName),
        ProfileIntf::getDeviceModel(profileName)};
}

auto DeviceFactory::create(sdbusplus::async::context& ctx,
                           const config::DeviceFactoryConfig& config,
                           PortIntf& serialPort, EventIntf::Events& events)
    -> std::unique_ptr<BaseDevice>
{
    switch (config.deviceType)
    {
        case ProfileIntf::DeviceType::reservoirPumpUnit:
            return std::make_unique<ReservoirPumpUnit>(ctx, config, serialPort,
                                                       events);
        case ProfileIntf::DeviceType::powerMonitorModule:
            return std::make_unique<PowerMonitorModule>(ctx, config, serialPort,
                                                        events);
        default:
            break;
    }

    return nullptr;
}

} // namespace phosphor::modbus::rtu::device
