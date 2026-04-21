#pragma once

#include "base_device.hpp"

namespace phosphor::modbus::rtu::device
{

class ReservoirPumpUnit : public BaseDevice
{
  public:
    explicit ReservoirPumpUnit(sdbusplus::async::context& ctx,
                               const config::Config& config,
                               PortIntf& serialPort, EventIntf::Events& events);
};

} // namespace phosphor::modbus::rtu::device
