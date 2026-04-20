#include "power_monitor_module.hpp"

#include <phosphor-logging/lg2.hpp>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

static constexpr auto PanasonicBJBPM102A0001PMMInterface =
    "xyz.openbmc_project.Configuration.PanasonicBJBPM102A0001PMM";
static constexpr auto Artesyn7000433970000PMMInterface =
    "xyz.openbmc_project.Configuration.Artesyn7000433970000PMM";
static constexpr auto DeltaECD70000020PMMInterface =
    "xyz.openbmc_project.Configuration.DeltaECD70000020PMM";

PowerMonitorModule::PowerMonitorModule(
    sdbusplus::async::context& ctx, const config::Config& config,
    PortIntf& serialPort, EventIntf::Events& events) :
    BaseDevice(ctx, config, serialPort, events)
{
    info("Power monitor module {NAME} created successfully", "NAME",
         config.name);
}

auto PowerMonitorModule::getInterfaces() -> std::unordered_set<std::string>
{
    return {PanasonicBJBPM102A0001PMMInterface,
            Artesyn7000433970000PMMInterface, DeltaECD70000020PMMInterface};
}

} // namespace phosphor::modbus::rtu::device
