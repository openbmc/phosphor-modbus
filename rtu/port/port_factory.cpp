#include "port_factory.hpp"

#include "usb_port.hpp"

#include <xyz/openbmc_project/Configuration/USBPort/client.hpp>

namespace phosphor::modbus::rtu::port
{

using USBPortConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::USBPort<>;

auto PortFactory::getInterfaces() -> std::vector<std::string>
{
    return {USBPortConfigIntf::interface};
}

auto PortFactory::getConfig(sdbusplus::async::context& ctx,
                            const sdbusplus::message::object_path& objectPath,
                            const std::string& interfaceName)
    -> sdbusplus::async::task<std::unique_ptr<config::PortFactoryConfig>>
{
    if (interfaceName == USBPortConfigIntf::interface)
    {
        co_return co_await USBPort::getConfig(ctx, objectPath);
    }

    co_return std::nullptr_t();
}

auto PortFactory::create(sdbusplus::async::context& ctx,
                         const config::PortFactoryConfig& config)
    -> std::unique_ptr<BasePort>
{
    if (config.portType == config::PortType::usb)
    {
        return std::make_unique<USBPort>(ctx, config);
    }

    return nullptr;
}

} // namespace phosphor::modbus::rtu::port
