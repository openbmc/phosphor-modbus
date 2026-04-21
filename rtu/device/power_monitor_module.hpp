#pragma once

#include "base_device.hpp"

namespace phosphor::modbus::rtu::device
{

class PowerMonitorModule : public BaseDevice
{
  public:
    explicit PowerMonitorModule(
        sdbusplus::async::context& ctx, const config::Config& config,
        PortIntf& serialPort, EventIntf::Events& events);
};

} // namespace phosphor::modbus::rtu::device
