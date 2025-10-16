#include "device_manager.hpp"

#include "device/device_factory.hpp"
#include "port/port_factory.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/server/manager.hpp>
#include <xyz/openbmc_project/Configuration/ModbusRTUDetect/client.hpp>

PHOSPHOR_LOG2_USING;

namespace phosphor::modbus::rtu
{

using ModbusRTUDetectIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::ModbusRTUDetect<>;
using DeviceFactoryIntf = phosphor::modbus::rtu::device::DeviceFactory;

static entity_manager::interface_list_t getInterfaces()
{
    entity_manager::interface_list_t interfaces;

    auto portInterfaces = PortIntf::PortFactory::getInterfaces();
    interfaces.insert(interfaces.end(), portInterfaces.begin(),
                      portInterfaces.end());

    interfaces.emplace_back(ModbusRTUDetectIntf::interface);

    auto deviceInterfaces = DeviceFactoryIntf::getInterfaces();
    interfaces.insert(interfaces.end(), deviceInterfaces.begin(),
                      deviceInterfaces.end());

    return interfaces;
}

DeviceManager::DeviceManager(sdbusplus::async::context& ctx) :
    ctx(ctx),
    entityManager(ctx, getInterfaces(),
                  std::bind_front(&DeviceManager::processConfigAdded, this),
                  std::bind_front(&DeviceManager::processConfigRemoved, this))
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
    if (interfaceName == ModbusRTUDetectIntf::interface && ports.size() == 0)
    {
        warning(
            "Skip processing ModbusRTUDetectIntf::interface as no serial ports detected yet");
        co_return;
    }

    auto portInterfaces = PortIntf::PortFactory::getInterfaces();
    if (std::find(portInterfaces.begin(), portInterfaces.end(),
                  interfaceName) != portInterfaces.end())
    {
        co_return co_await processPortAdded(objectPath, interfaceName);
    }

    if (interfaceName == ModbusRTUDetectIntf::interface)
    {
        co_return co_await processInventoryAdded(objectPath);
    }

    auto deviceInterfaces = DeviceFactoryIntf::getInterfaces();
    if (std::find(deviceInterfaces.begin(), deviceInterfaces.end(),
                  interfaceName) != deviceInterfaces.end())
    {
        co_return co_await processDeviceAdded(objectPath, interfaceName);
    }
}

auto DeviceManager::processPortAdded(
    const sdbusplus::message::object_path& objectPath,
    const std::string& interfaceName) -> sdbusplus::async::task<>
{
    auto config = co_await PortIntf::PortFactory::getConfig(
        ctx, objectPath, interfaceName);
    if (!config)
    {
        error("Failed to get Port config for {PATH}", "PATH", objectPath);
        co_return;
    }

    try
    {
        ports[config->name] = PortIntf::PortFactory::create(ctx, *config);
    }
    catch (const std::exception& e)
    {
        error("Failed to create Port for {PATH} with {ERROR}", "PATH",
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
            std::make_unique<InventoryIntf::Device>(ctx, config, ports);
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
        co_await DeviceFactoryIntf::getConfig(ctx, objectPath, interfaceName);
    if (!res)
    {
        error("Failed to get Device config for {PATH}", "PATH", objectPath);
        co_return;
    }
    auto config = res.value();

    try
    {
        auto serialPort = ports.find(config.portName);
        if (serialPort == ports.end())
        {
            error("Failed to get Inventory Device config for {PATH}", "PATH",
                  objectPath);
            co_return;
        }

        auto device =
            DeviceFactoryIntf::create(ctx, config, *(serialPort->second));
        ctx.spawn(device->readSensorRegisters());
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
