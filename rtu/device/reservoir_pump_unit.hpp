#pragma once

#include "base_device.hpp"

namespace phosphor::modbus::rtu::device
{

namespace config
{

struct DeviceFactoryConfig;

} // namespace config

class ReservoirPumpUnit : public BaseDevice
{
  public:
    explicit ReservoirPumpUnit(sdbusplus::async::context& ctx,
                               const config::Config& config,
                               const std::unique_ptr<PortIntf>& serialPort,
                               EventIntf::Events& events);

    static auto getInterfaces() -> std::vector<std::string>;

    static auto getConfig(sdbusplus::async::context& ctx,
                          const sdbusplus::message::object_path& objectPath,
                          const std::string& interfaceName)
        -> sdbusplus::async::task<std::optional<config::DeviceFactoryConfig>>;
};

} // namespace phosphor::modbus::rtu::device
