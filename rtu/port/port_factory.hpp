#pragma once

#include "base_port.hpp"

#include <sdbusplus/async.hpp>

#include <string>
#include <vector>

namespace phosphor::modbus::rtu::port
{

namespace config
{

enum class PortType
{
    usb,
    unknown
};

struct PortFactoryConfig : public Config
{
    PortType portType = PortType::unknown;

    virtual ~PortFactoryConfig() = default;
};

} // namespace config

class PortFactory
{
  public:
    static auto getInterfaces() -> std::vector<std::string>;

    static auto getConfig(sdbusplus::async::context& ctx,
                          const sdbusplus::message::object_path& objectPath,
                          const std::string& interfaceName)
        -> sdbusplus::async::task<std::unique_ptr<config::PortFactoryConfig>>;

    static auto create(sdbusplus::async::context& ctx,
                       const config::PortFactoryConfig& config)
        -> std::unique_ptr<BasePort>;
};

} // namespace phosphor::modbus::rtu::port
