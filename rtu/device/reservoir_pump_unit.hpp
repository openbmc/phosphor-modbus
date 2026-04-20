#pragma once

#include "base_device.hpp"

#include <unordered_set>

namespace phosphor::modbus::rtu::device
{

class ReservoirPumpUnit : public BaseDevice
{
  public:
    explicit ReservoirPumpUnit(sdbusplus::async::context& ctx,
                               const config::Config& config,
                               PortIntf& serialPort, EventIntf::Events& events);

    static auto getInterfaces() -> std::unordered_set<std::string>;
};

} // namespace phosphor::modbus::rtu::device
