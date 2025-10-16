#include "device_factory.hpp"

#include "reservoir_pump_unit.hpp"

#include <string>
#include <vector>

namespace phosphor::modbus::rtu::device
{

using ReservoirPumpUnitIntf = phosphor::modbus::rtu::device::ReservoirPumpUnit;

auto DeviceFactory::getInterfaces() -> std::vector<std::string>
{
    std::vector<std::string> interfaces{};

    auto rpuInterfaces = ReservoirPumpUnitIntf::getInterfaces();
    interfaces.insert(interfaces.end(), rpuInterfaces.begin(),
                      rpuInterfaces.end());

    return interfaces;
}

auto DeviceFactory::getConfig(sdbusplus::async::context& ctx,
                              const sdbusplus::message::object_path& objectPath,
                              const std::string& interfaceName)
    -> sdbusplus::async::task<std::optional<config::DeviceFactoryConfig>>
{
    auto rpuInterfaces = ReservoirPumpUnitIntf::getInterfaces();
    if (rpuInterfaces.find(interfaceName) != rpuInterfaces.end())
    {
        co_return co_await ReservoirPumpUnitIntf::getConfig(ctx, objectPath,
                                                            interfaceName);
    }

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
        default:
            break;
    }

    return nullptr;
}

} // namespace phosphor::modbus::rtu::device
