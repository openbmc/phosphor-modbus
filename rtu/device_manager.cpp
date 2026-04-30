#include "device_manager.hpp"

#include "device/device_factory.hpp"
#include "modbus_rtu_config.hpp"
#include "port/port_factory.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/server/manager.hpp>

PHOSPHOR_LOG2_USING;

namespace phosphor::modbus::rtu
{

using DeviceFactoryIntf = phosphor::modbus::rtu::device::DeviceFactory;
using DeviceFactoryConfigIntf =
    phosphor::modbus::rtu::device::config::DeviceFactoryConfig;

static entity_manager::interface_list_t getInterfaces()
{
    entity_manager::interface_list_t interfaces;

    auto portInterfaces = PortIntf::PortFactory::getInterfaces();
    interfaces.insert(interfaces.end(), portInterfaces.begin(),
                      portInterfaces.end());

    auto deviceInterfaces = DeviceFactoryIntf::getInterfaces();
    interfaces.insert(interfaces.end(), deviceInterfaces.begin(),
                      deviceInterfaces.end());

    return interfaces;
}

DeviceManager::DeviceManager(sdbusplus::async::context& ctx) :
    ctx(ctx),
    entityManager(ctx, getInterfaces(),
                  std::bind_front(&DeviceManager::processConfigAdded, this),
                  std::bind_front(&DeviceManager::processConfigRemoved, this)),
    events(ctx)
{
    ctx.spawn(entityManager.handleInventoryGet());
    ctx.spawn(cleanupStoppedDevices());
    info("DeviceManager created successfully");
}

auto DeviceManager::processConfigAdded(const sdbusplus::object_path& objectPath,
                                       const std::string& interfaceName)
    -> sdbusplus::async::task<>
{
    debug("Config added for {PATH} with {INTF}", "PATH", objectPath, "INTF",
          interfaceName);

    auto portInterfaces = PortIntf::PortFactory::getInterfaces();
    if (std::find(portInterfaces.begin(), portInterfaces.end(),
                  interfaceName) != portInterfaces.end())
    {
        co_return co_await processPortAdded(objectPath, interfaceName);
    }

    auto deviceInterfaces = DeviceFactoryIntf::getInterfaces();
    if (std::find(deviceInterfaces.begin(), deviceInterfaces.end(),
                  interfaceName) != deviceInterfaces.end())
    {
        if (ports.empty())
        {
            warning("Skip processing {INTF} as no serial ports detected yet",
                    "INTF", interfaceName);
            co_return;
        }
        co_return co_await processInventoryAdded(objectPath, interfaceName);
    }
}

auto DeviceManager::processPortAdded(const sdbusplus::object_path& objectPath,
                                     const std::string& interfaceName)
    -> sdbusplus::async::task<>
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
    const sdbusplus::object_path& objectPath, const std::string& interfaceName)
    -> sdbusplus::async::task<>
{
    auto config =
        co_await DeviceFactoryIntf::getConfig(ctx, objectPath, interfaceName);
    if (!config)
    {
        error("Failed to get config for {PATH}", "PATH", objectPath);
        co_return;
    }

    auto inventoryIter = inventoryDevices.find(config->name);
    if (inventoryIter != inventoryDevices.end() &&
        !inventoryIter->second->isStopped())
    {
        debug("Inventory device {NAME} already exists, skipping", "NAME",
              config->name);
        co_return;
    }

    auto portIter = ports.find(config->serialPort);
    if (portIter == ports.end())
    {
        error("Serial port {PORT} not found for {NAME}", "PORT",
              config->serialPort, "NAME", config->name);
        co_return;
    }

    auto callback =
        [this, config = *config](bool success) -> sdbusplus::async::task<> {
        if (success)
        {
            co_await processDeviceAdded(config);
        }
        else
        {
            auto iter = devices.find(config.name);
            if (iter != devices.end())
            {
                iter->second->requestStop();
                debug("Requested sensor device {NAME} to stop", "NAME",
                      config.name);
            }
        }
    };

    try
    {
        auto inventoryDevice = std::make_unique<InventoryIntf::Device>(
            ctx, *config, *(portIter->second), std::move(callback));
        ctx.spawn(inventoryDevice->startProbing());
        inventoryDevices[config->name] = std::move(inventoryDevice);
    }
    catch (const std::exception& e)
    {
        error("Failed to create Inventory Device for {PATH} with {ERROR}",
              "PATH", objectPath, "ERROR", e);
        co_return;
    }
}

auto DeviceManager::processDeviceAdded(const DeviceFactoryConfigIntf& config)
    -> sdbusplus::async::task<>
{
    auto iter = devices.find(config.name);
    if (iter != devices.end())
    {
        if (iter->second->isStopped())
        {
            info("Restarting sensor device {NAME}", "NAME", config.name);
            iter->second->restart();
            ctx.spawn(iter->second->readSensorRegisters());
        }
        else
        {
            debug("Device {NAME} already exists, skipping", "NAME",
                  config.name);
        }
        co_return;
    }

    auto portIter = ports.find(config.serialPort);
    if (portIter == ports.end())
    {
        error("Serial port {PORT} not found for {NAME}", "PORT",
              config.serialPort, "NAME", config.name);
        co_return;
    }

    try
    {
        auto device =
            DeviceFactoryIntf::create(ctx, config, *(portIter->second), events);
        ctx.spawn(device->readSensorRegisters());
        devices[config.name] = std::move(device);
    }
    catch (const std::exception& e)
    {
        error("Failed to create Device for {NAME} with {ERROR}", "NAME",
              config.name, "ERROR", e);
        co_return;
    }
}

auto DeviceManager::cleanupStoppedDevices() -> sdbusplus::async::task<>
{
    while (!ctx.stop_requested())
    {
        co_await sdbusplus::async::sleep_for(ctx, deviceCleanupInterval);

        std::erase_if(devices, [](const auto& entry) {
            if (entry.second->isStopped())
            {
                info("Removing stopped sensor device {NAME}", "NAME",
                     entry.first);
                return true;
            }
            return false;
        });

        std::erase_if(inventoryDevices, [](const auto& entry) {
            if (entry.second->isStopped())
            {
                info("Removing stopped inventory device {NAME}", "NAME",
                     entry.first);
                return true;
            }
            return false;
        });
    }
}

auto DeviceManager::processConfigRemoved(
    const sdbusplus::object_path& objectPath,
    const std::string& /*interfaceName*/) -> sdbusplus::async::task<>
{
    auto name = objectPath.filename();
    info("Config removed for device {NAME}", "NAME", name);

    // Stop inventory device — its probing coroutine will clean up the
    // inventory D-Bus object and stop the sensor device via the probe
    // callback. Both are cleaned up by the regular cleanup loop.
    auto inventoryIter = inventoryDevices.find(name);
    if (inventoryIter != inventoryDevices.end())
    {
        inventoryIter->second->requestStop();
    }
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
