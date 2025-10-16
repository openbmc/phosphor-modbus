#include "reservoir_pump_unit.hpp"

#include "device_factory.hpp"

#include <phosphor-logging/lg2.hpp>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

static constexpr auto ModbusRDF040DSS5193E0ReservoirPumpUnitInterface =
    "xyz.openbmc_project.Configuration.ModbusRDF040DSS5193E0ReservoirPumpUnit";

static const std::unordered_map<std::string_view, config::DeviceModel>
    validDevices = {{ModbusRDF040DSS5193E0ReservoirPumpUnitInterface,
                     config::DeviceModel::RDF040DSS5193E0}};

ReservoirPumpUnit::ReservoirPumpUnit(
    sdbusplus::async::context& ctx, const config::Config& config,
    const std::unique_ptr<PortIntf>& serialPort) :
    BaseDevice(ctx, config, serialPort)
{
    info("USB port {NAME} created successfully", "NAME", config.name);
}

auto ReservoirPumpUnit::getInterfaces() -> std::vector<std::string>
{
    return {ModbusRDF040DSS5193E0ReservoirPumpUnitInterface};
}

/*
auto updateBaseConfig(sdbusplus::async::context& ctx,
                      const sdbusplus::message::object_path& objectPath,
                      const std::string& interfaceName, Config& config) -> bool;
*/

auto ReservoirPumpUnit::getConfig(
    sdbusplus::async::context& ctx,
    const sdbusplus::message::object_path& objectPath,
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

    config.deviceType = config::DeviceType::reservoirPumpUnit;

    /*

    auto baseConfig =
        co_await config::getConfig(ctx, objectPath, interfaceName);

    if (!baseConfig)
    {
        co_return std::nullopt;
    }
        */

    co_return config;
}

} // namespace phosphor::modbus::rtu::device
