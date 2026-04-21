#pragma once

#include "base_device.hpp"

namespace phosphor::modbus::rtu::device
{

namespace config
{

using ConfigIntf::Config;
using ConfigIntf::getConfig;

struct DeviceFactoryConfig : public Config
{
    ProfileIntf::DeviceType deviceType = ProfileIntf::DeviceType::unknown;
    ProfileIntf::DeviceModel deviceModel = ProfileIntf::DeviceModel::unknown;
};

} // namespace config

class DeviceFactory
{
  public:
    DeviceFactory() = delete;

    static auto getInterfaces() -> std::vector<std::string>;

    static auto getConfig(sdbusplus::async::context& ctx,
                          const sdbusplus::object_path& objectPath,
                          const std::string& interfaceName)
        -> sdbusplus::async::task<std::optional<config::DeviceFactoryConfig>>;

    static auto create(sdbusplus::async::context& ctx,
                       const config::DeviceFactoryConfig& config,
                       PortIntf& serialPort, EventIntf::Events& events)
        -> std::unique_ptr<BaseDevice>;
};

} // namespace phosphor::modbus::rtu::device
