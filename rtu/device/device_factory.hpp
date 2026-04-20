#pragma once

#include "base_device.hpp"

namespace phosphor::modbus::rtu::device
{

namespace config
{

using ConfigIntf::Config;
using ConfigIntf::getConfig;

enum class DeviceType
{
    reservoirPumpUnit,
    heatExchanger,
    flowMeter,
    powerMonitorModule,
    unknown
};

enum class DeviceModel
{
    RDF040DSS5193E0,
    PanasonicBJBPM102A0001,
    Artesyn7000433970000,
    DeltaECD70000020,
    unknown
};

struct DeviceFactoryConfig : public Config
{
    DeviceType deviceType = DeviceType::unknown;
    DeviceModel deviceModel = DeviceModel::unknown;
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
