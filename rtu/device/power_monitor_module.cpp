#include "power_monitor_module.hpp"

#include <phosphor-logging/lg2.hpp>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

PowerMonitorModule::PowerMonitorModule(
    sdbusplus::async::context& ctx, const config::Config& config,
    PortIntf& serialPort, EventIntf::Events& events) :
    BaseDevice(ctx, config, serialPort, events)
{
    info("Power monitor module {NAME} created successfully", "NAME",
         config.name);
}

} // namespace phosphor::modbus::rtu::device
