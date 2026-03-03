#include "usb_port.hpp"

#include "common/entity_manager_interface.hpp"
#include "port_factory.hpp"

#include <fcntl.h>

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Configuration/USBPort/client.hpp>

#include <filesystem>
#include <format>
#include <optional>
#include <regex>

namespace phosphor::modbus::rtu::port
{

PHOSPHOR_LOG2_USING;

using USBPortConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::USBPort<>;

namespace config
{

struct USBPortConfig : public PortFactoryConfig
{
    std::string address = "unknown";
    uint16_t port = 0;
    uint16_t interface = 0;
    uint16_t hubport = 0;

    virtual ~USBPortConfig() = default;
};

} // namespace config

static auto getDevicePath(const config::PortFactoryConfig& inConfig)
    -> std::string
{
    namespace fs = std::filesystem;
    auto config = static_cast<const config::USBPortConfig&>(inConfig);

    std::string spattern;

    info("Hub port ID passed {HUBPORT}", "HUBPORT", config.hubport);
    if (config.hubport)
    {
        // platform-1e6a3000.usb-usb-0:1.2:1.0-port0
        // platform-1e6a3000.usb-usb-0:1.1:1.0-port0
        spattern = std::format("platform-{}\\.usb-usb-[0-9]:[0-9]\\.{}:[0-9]\\.{}-port{}", config.address, config.hubport, config.interface, config.port);
    }
    else
    {
        // platform-1e6a1000.usb-usb-0:1:1.0-port0
        spattern = std::format("platform-{}\\.usb-usb-[0-9]:[0-9]:[0-9]\\.{}-port{}", config.address, config.interface, config.port);
    }
    std::regex pattern(spattern);

    fs::path searchDir = "/dev/serial/by-path/";

    for (const auto& entry : fs::recursive_directory_iterator(searchDir))
    {
        if (entry.is_symlink())
        {
            auto filePath = entry.path();
            if (std::regex_search(filePath.filename().string(), pattern))
            {
                return ("/dev/" +
                        fs::read_symlink(filePath).filename().string());
            }
        }
    }

    throw std::runtime_error("Failed to get device path");
}

USBPort::USBPort(sdbusplus::async::context& ctx,
                 const config::PortFactoryConfig& config) :
    BasePort(ctx, config, getDevicePath(config))
{
    info("USB port {NAME} created successfully", "NAME", config.name);
}

auto USBPort::getConfig(sdbusplus::async::context& ctx,
                        const sdbusplus::message::object_path& objectPath)
    -> sdbusplus::async::task<std::unique_ptr<config::PortFactoryConfig>>
{
    auto config = std::make_unique<config::USBPortConfig>();
    if (!config)
    {
        co_return std::nullptr_t();
    }

    auto properties =
        co_await USBPortConfigIntf(ctx)
            .service(entity_manager::EntityManagerInterface::serviceName)
            .path(objectPath.str)
            .properties();

    auto res = updateBaseConfig(*config, properties);
    if (!res)
    {
        co_return std::nullptr_t();
    }

    config->address = properties.device_address;
    config->port = properties.port;
    config->hubport = properties.hub_port;
    config->interface = properties.device_interface;
    config->portType = config::PortType::usb;

    debug(
        "USB port config: {NAME} {PORT_TYPE} {PORT_MODE} {ADDRESS} {PORT} {INTERFACE} {BAUD_RATE} {RTS_DELAY}",
        "NAME", config->name, "PORT_TYPE", config->portType, "PORT_MODE",
        config->portMode, "ADDRESS", config->address, "PORT", config->port,
        "INTERFACE", config->interface, "BAUD_RATE", config->baudRate,
        "RTS_DELAY", config->rtsDelay);

    co_return config;
}

} // namespace phosphor::modbus::rtu::port
