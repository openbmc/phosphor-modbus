#pragma once

#include "base_port.hpp"

#include <sdbusplus/async.hpp>

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
                          const sdbusplus::message::object_path& objectPath)
        -> sdbusplus::async::task<std::unique_ptr<config::PortFactoryConfig>>;
};

} // namespace phosphor::modbus::rtu::port
