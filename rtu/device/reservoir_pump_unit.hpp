#pragma once

#include "base_device.hpp"

#include <unordered_set>

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
                               PortIntf& serialPort);

    static auto getInterfaces() -> std::unordered_set<std::string>;

    static auto getConfig(sdbusplus::async::context& ctx,
                          const sdbusplus::message::object_path& objectPath,
                          const std::string& interfaceName)
        -> sdbusplus::async::task<std::optional<config::DeviceFactoryConfig>>;
};

} // namespace phosphor::modbus::rtu::device
