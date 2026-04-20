#include "device_factory.hpp"

#include "power_monitor_module.hpp"
#include "reservoir_pump_unit.hpp"

#include <phosphor-logging/lg2.hpp>

#include <string>
#include <vector>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

using PowerMonitorModuleIntf =
    phosphor::modbus::rtu::device::PowerMonitorModule;
using ReservoirPumpUnitIntf = phosphor::modbus::rtu::device::ReservoirPumpUnit;

auto DeviceFactory::getInterfaces() -> std::vector<std::string>
{
    std::vector<std::string> interfaces{};

    auto rpuInterfaces = ReservoirPumpUnitIntf::getInterfaces();
    interfaces.insert(interfaces.end(), rpuInterfaces.begin(),
                      rpuInterfaces.end());

    auto pmmInterfaces = PowerMonitorModuleIntf::getInterfaces();
    interfaces.insert(interfaces.end(), pmmInterfaces.begin(),
                      pmmInterfaces.end());

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

    auto rpuInterfaces = ReservoirPumpUnitIntf::getInterfaces();
    if (rpuInterfaces.contains(interfaceName))
    {
        co_return config::DeviceFactoryConfig{
            {std::move(*baseConfig)}, config::DeviceType::reservoirPumpUnit};
    }

    auto pmmInterfaces = PowerMonitorModuleIntf::getInterfaces();
    if (pmmInterfaces.contains(interfaceName))
    {
        co_return config::DeviceFactoryConfig{
            {std::move(*baseConfig)}, config::DeviceType::powerMonitorModule};
    }

    error("Unknown device interface {INTF}", "INTF", interfaceName);
    co_return std::nullopt;
}

auto DeviceFactory::create(sdbusplus::async::context& ctx,
                           const config::DeviceFactoryConfig& config,
                           PortIntf& serialPort, EventIntf::Events& events)
    -> std::unique_ptr<BaseDevice>
{
    switch (config.deviceType)
    {
        case config::DeviceType::reservoirPumpUnit:
            return std::make_unique<ReservoirPumpUnit>(ctx, config, serialPort,
                                                       events);
        case config::DeviceType::powerMonitorModule:
            return std::make_unique<PowerMonitorModule>(ctx, config, serialPort,
                                                        events);
        default:
            break;
    }

    return nullptr;
}

} // namespace phosphor::modbus::rtu::device
