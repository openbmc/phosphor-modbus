#include "power_monitor_module.hpp"

#include "device_factory.hpp"

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

static const std::unordered_map<std::string_view, config::DeviceModel>
    validDevices = {
        {PanasonicBJBPM102A0001PMMInterface,
         config::DeviceModel::PanasonicBJBPM102A0001},
        {Artesyn7000433970000PMMInterface,
         config::DeviceModel::Artesyn7000433970000},
        {DeltaECD70000020PMMInterface, config::DeviceModel::DeltaECD70000020}};

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

auto PowerMonitorModule::getConfig(sdbusplus::async::context& ctx,
                                   const sdbusplus::object_path& objectPath,
                                   const std::string& interfaceName)
    -> sdbusplus::async::task<std::optional<config::DeviceFactoryConfig>>
{
    config::DeviceFactoryConfig config{};

    auto res = co_await config::updateBaseConfig(ctx, objectPath, interfaceName,
                                                 config);
    if (!res)
    {
        co_return std::nullopt;
    }

    for (const auto& [deviceInterface, deviceModel] : validDevices)
    {
        if (interfaceName == deviceInterface)
        {
            config.deviceModel = deviceModel;
        }
    }

    if (config.deviceModel == config::DeviceModel::unknown)
    {
        error("Invalid device model {MODEL} for {NAME}", "MODEL", interfaceName,
              "NAME", config.name);
        co_return std::nullopt;
    }

    config.deviceType = config::DeviceType::powerMonitorModule;

    co_return config;
}

} // namespace phosphor::modbus::rtu::device
