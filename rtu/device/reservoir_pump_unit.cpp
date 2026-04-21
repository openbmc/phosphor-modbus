#include "reservoir_pump_unit.hpp"

#include <phosphor-logging/lg2.hpp>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

ReservoirPumpUnit::ReservoirPumpUnit(
    sdbusplus::async::context& ctx, const config::Config& config,
    PortIntf& serialPort, EventIntf::Events& events) :
    BaseDevice(ctx, config, serialPort, events)
{
    info("Reservoir pump unit {NAME} created successfully", "NAME",
         config.name);
}

} // namespace phosphor::modbus::rtu::device
