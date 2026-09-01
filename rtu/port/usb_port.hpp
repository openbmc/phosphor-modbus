#pragma once

#include "base_port.hpp"

#include <sdbusplus/async.hpp>

#include <string>

namespace phosphor::modbus::rtu::port
{

namespace config
{

struct PortFactoryConfig;

} // namespace config

class USBPort : public BasePort
{
  public:
    explicit USBPort(sdbusplus::async::context& ctx,
                     const config::PortFactoryConfig& config);

    static auto getConfig(sdbusplus::async::context& ctx,
                          const sdbusplus::object_path& objectPath)
        -> sdbusplus::async::task<std::unique_ptr<config::PortFactoryConfig>>;

    /** @brief The serial device the port config resolves to.
     *  @throws std::runtime_error if no matching device is present. */
    static auto getDevicePath(const config::PortFactoryConfig& config)
        -> std::string;
};

} // namespace phosphor::modbus::rtu::port
