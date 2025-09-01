#include "device_manager.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/server/manager.hpp>
#include <xyz/openbmc_project/Configuration/ModbusRTUDetect/client.hpp>
#include <xyz/openbmc_project/Configuration/USBPort/client.hpp>

PHOSPHOR_LOG2_USING;

namespace phosphor::modbus::rtu
{

using USBPortConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::USBPort<>;
using ModbusRTUDetectIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::ModbusRTUDetect<>;
namespace DeviceConfigIntf = phosphor::modbus::rtu::device::config;

const entity_manager::interface_list_t interfaces{
    USBPortConfigIntf::interface, ModbusRTUDetectIntf::interface,
    DeviceConfigIntf::ModbusRDF040DSS5193E0ReservoirPumpUnitInterface,
    DeviceConfigIntf::ModbusRDF040DSS5193E0HeatExchangerInterface};

DeviceManager::DeviceManager(sdbusplus::async::context& ctx) :
    ctx(ctx),
    entityManager(ctx, interfaces,
                  std::bind_front(&DeviceManager::processConfigAdded, this),
                  std::bind_front(&DeviceManager::processConfigRemoved, this)),
    events(ctx)
{
    ctx.spawn(entityManager.handleInventoryGet());
    info("DeviceManager created successfully");
}

auto DeviceManager::processConfigAdded(
    const sdbusplus::message::object_path& objectPath,
    const std::string& interfaceName) -> sdbusplus::async::task<>
{
    debug("Config added for {PATH} with {INTF}", "PATH", objectPath, "INTF",
          interfaceName);
    if (interfaceName == ModbusRTUDetectIntf::interface &&
        serialPorts.size() == 0)
    {
        warning(
            "Skip processing ModbusRTUDetectIntf::interface as no serial ports detected yet");
        co_return;
    }

    if (interfaceName == USBPortConfigIntf::interface)
    {
        co_await processPortAdded(objectPath);
    }
    else if (interfaceName == ModbusRTUDetectIntf::interface)
    {
        co_await processInventoryAdded(objectPath);
    }
    else if (interfaceName ==
                 DeviceConfigIntf::
                     ModbusRDF040DSS5193E0ReservoirPumpUnitInterface ||
             interfaceName ==
                 DeviceConfigIntf::ModbusRDF040DSS5193E0HeatExchangerInterface)
    {
        co_await processDeviceAdded(objectPath, interfaceName);
    }
}

auto DeviceManager::processPortAdded(
    const sdbusplus::message::object_path& objectPath)
    -> sdbusplus::async::task<>
{
    auto res = co_await SerialPortIntf::config::getConfig(ctx, objectPath);
    if (!res)
    {
        error("Failed to get Serial Port config for {PATH}", "PATH",
              objectPath);
        co_return;
    }
    auto config = res.value();
    try
    {
        serialPorts[config.name] =
            std::make_unique<SerialPortIntf::SerialPort>(ctx, config);
    }
    catch (const std::exception& e)
    {
        error("Failed to create Serial Port for {PATH} with {ERROR}", "PATH",
              objectPath, "ERROR", e);
        co_return;
    }
}

auto DeviceManager::processInventoryAdded(
    const sdbusplus::message::object_path& objectPath)
    -> sdbusplus::async::task<>
{
    auto res = co_await InventoryIntf::config::getConfig(ctx, objectPath);
    if (!res)
    {
        error("Failed to get Inventory Device config for {PATH}", "PATH",
              objectPath);
        co_return;
    }
    auto config = res.value();
    try
    {
        auto inventoryDevice =
            std::make_unique<InventoryIntf::Device>(ctx, config, serialPorts);
        ctx.spawn(inventoryDevice->probePorts());
        inventoryDevices[config.name] = std::move(inventoryDevice);
    }
    catch (const std::exception& e)
    {
        error("Failed to create Inventory Device for {PATH} with {ERROR}",
              "PATH", objectPath, "ERROR", e);
        co_return;
    }
}

auto DeviceManager::processDeviceAdded(
    const sdbusplus::message::object_path& objectPath,
    const std::string& interfaceName) -> sdbusplus::async::task<>
{
    auto res =
        co_await DeviceConfigIntf::getConfig(ctx, objectPath, interfaceName);
    if (!res)
    {
        error("Failed to get Device config for {PATH}", "PATH", objectPath);
        co_return;
    }
    auto config = res.value();

    auto deviceInfo = getDeviceInfoFromInventory(config.name, config.address,
                                                 config.portName);
    if (!deviceInfo)
    {
        error("Failed to get Device info for {PATH}", "PATH", objectPath);
        co_return;
    }

    try
    {
        auto serialPort = serialPorts.find(config.portName);
        if (serialPort == serialPorts.end())
        {
            error("Failed to find Serial Port for device creation for {PATH}",
                  "PATH", objectPath);
            co_return;
        }
        config.parity = deviceInfo.value().first;
        config.baudRate = deviceInfo.value().second;
        auto device = std::make_unique<DeviceIntf::Device>(
            ctx, config, serialPort->second, events);
        devices[config.name] = std::move(device);
    }
    catch (const std::exception& e)
    {
        error("Failed to create Device for {PATH} with {ERROR}", "PATH",
              objectPath, "ERROR", e);
        co_return;
    }
}

auto DeviceManager::processConfigRemoved(
    const sdbusplus::message::object_path& /*unused*/,
    const std::string& /*unused*/) -> sdbusplus::async::task<>
{
    // TODO: Implement this
    co_return;
}

auto DeviceManager::getDeviceInfoFromInventory(
    const std::string& deviceName, uint8_t address, std::string& portName)
    -> std::optional<device_info_t>
{
    std::string deviceSuffix = "_" + std::to_string(address) + "_" + portName;
    size_t pos = deviceName.find(deviceSuffix);

    if (pos == std::string::npos)
    {
        return std::nullopt;
    }

    std::string inventoryName = deviceName.substr(0, pos);

    const auto& inventoryDevice = inventoryDevices.find(inventoryName);

    if (inventoryDevice == inventoryDevices.end())
    {
        return std::nullopt;
    }

    return device_info_t{inventoryDevice->second->config.parity,
                         inventoryDevice->second->config.baudRate};
}

} // namespace phosphor::modbus::rtu

auto main() -> int
{
    constexpr auto path = "/xyz/openbmc_project";
    constexpr auto serviceName = "xyz.openbmc_project.ModbusRTU";
    sdbusplus::async::context ctx;
    sdbusplus::server::manager_t manager{ctx, path};

    info("Creating Modbus device manager at {PATH}", "PATH", path);
    phosphor::modbus::rtu::DeviceManager deviceManager{ctx};

    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
