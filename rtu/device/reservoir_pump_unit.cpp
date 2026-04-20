#include "reservoir_pump_unit.hpp"

#include <phosphor-logging/lg2.hpp>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

static constexpr auto DeltaRDF040DSS5193E0ReservoirPumpUnitInterface =
    "xyz.openbmc_project.Configuration.DeltaRDF040DSS5193E0ReservoirPumpUnit";

ReservoirPumpUnit::ReservoirPumpUnit(
    sdbusplus::async::context& ctx, const config::Config& config,
    PortIntf& serialPort, EventIntf::Events& events) :
    BaseDevice(ctx, config, serialPort, events)
{
    info("Reservoir pump unit {NAME} created successfully", "NAME",
         config.name);
}

auto ReservoirPumpUnit::getInterfaces() -> std::unordered_set<std::string>
{
    return {DeltaRDF040DSS5193E0ReservoirPumpUnitInterface};
}

} // namespace phosphor::modbus::rtu::device
