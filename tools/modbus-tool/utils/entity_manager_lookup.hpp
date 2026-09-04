#pragma once

#include "base_config.hpp"
#include "port/port_factory.hpp"

#include <sdbusplus/async.hpp>

#include <memory>
#include <string>
#include <vector>

namespace modbus_tool
{

namespace ConfigIntf = phosphor::modbus::rtu::config;
namespace PortIntf = phosphor::modbus::rtu::port;

/** @brief The entity-manager configurations found for one device name.
 *
 *  A second sourced device carries a configuration for each variant on the
 *  same object, and only one of them is really present, so this holds all of
 *  them until a probe says which. Empty when the name is not configured. */
struct DeviceVariants
{
    std::string name;
    std::vector<ConfigIntf::Config> configs;
};

/** @brief A serial port's configuration and the device it resolves to. */
struct PortDetails
{
    std::unique_ptr<PortIntf::config::PortFactoryConfig> config;
    std::string devicePath;
};

/** @brief Look the named devices up, in the order given. */
auto lookupDevices(sdbusplus::async::context& ctx,
                   const std::vector<std::string>& names)
    -> sdbusplus::async::task<std::vector<DeviceVariants>>;

/** @brief Look a port up and resolve its serial device.
 *  @return An empty config if the port is not configured, or its device could
 *          not be resolved. */
auto lookupPort(sdbusplus::async::context& ctx, const std::string& portName)
    -> sdbusplus::async::task<PortDetails>;

} // namespace modbus_tool
