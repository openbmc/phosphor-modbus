#include "device_manager.hpp"

#include "device/device_factory.hpp"
#include "modbus_rtu_config.hpp"
#include "port/port_factory.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/server/manager.hpp>
#include <xyz/openbmc_project/Inventory/Item/client.hpp>
#include <xyz/openbmc_project/Metric/Value/client.hpp>
#include <xyz/openbmc_project/Sensor/Value/client.hpp>
#include <xyz/openbmc_project/Software/Version/client.hpp>

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
    events(ctx, STATE_DIR), allowedDevices(ctx, CONFIG_DIR)
{
    events.restore();

    ctx.spawn(discoverConfigs());
    ctx.spawn(cleanupStoppedDevices());
    allowedDevices.startWatching();
    info("DeviceManager created successfully");
}

auto DeviceManager::discoverConfigs() -> sdbusplus::async::task<>
{
    co_await entityManager.handleInventoryGet(
        PortIntf::PortFactory::getInterfaces());
    co_await entityManager.handleInventoryGet(
        DeviceFactoryIntf::getInterfaces());
}

auto DeviceManager::processConfigAdded(
    const sdbusplus::object_path& objectPath, const std::string& interfaceName,
    const entity_manager::ConfigData& inInterfaces) -> sdbusplus::async::task<>
{
    debug("Config added for {PATH} with {INTF}", "PATH", objectPath, "INTF",
          interfaceName);

    auto portInterfaces = PortIntf::PortFactory::getInterfaces();
    if (std::find(portInterfaces.begin(), portInterfaces.end(),
                  interfaceName) != portInterfaces.end())
    {
        // Ports use a targeted properties() fetch, so the map is not needed.
        co_return co_await processPortAdded(objectPath, interfaceName);
    }

    auto deviceInterfaces = DeviceFactoryIntf::getInterfaces();
    if (std::find(deviceInterfaces.begin(), deviceInterfaces.end(),
                  interfaceName) != deviceInterfaces.end())
    {
        co_return co_await processInventoryAdded(objectPath, interfaceName,
                                                 inInterfaces);
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

    // A port can be seen via both the get and an add signal; recreating it
    // would destroy the BasePort that devices already reference.
    if (ports.contains(config->name))
    {
        debug("Port {NAME} already created, skipping", "NAME", config->name);
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

    // Bind any device configs that arrived before this port existed.
    auto pendingIter = pendingDevices.find(config->name);
    if (pendingIter != pendingDevices.end())
    {
        auto pending = std::move(pendingIter->second);
        pendingDevices.erase(pendingIter);
        for (const auto& device : pending)
        {
            // No cached map for deferred devices - pass empty to re-fetch.
            co_await processInventoryAdded(device.objectPath,
                                           device.interfaceName, {});
        }
    }
}

auto DeviceManager::processInventoryAdded(
    const sdbusplus::object_path& objectPath, const std::string& interfaceName,
    const entity_manager::ConfigData& inInterfaces) -> sdbusplus::async::task<>
{
    auto config = co_await DeviceFactoryIntf::getConfig(
        ctx, objectPath, interfaceName, inInterfaces);
    if (!config)
    {
        error("Failed to get config for {PATH}", "PATH", objectPath);
        co_return;
    }

    auto& variants = inventoryDevices[config->name];
    auto variantIter = variants.find(config->type);
    if (variantIter != variants.end() && !variantIter->second->isStopped())
    {
        debug("Inventory device {NAME} already exists, skipping", "NAME",
              config->name);
        co_return;
    }

    auto portIter = ports.find(config->serialPort);
    if (portIter == ports.end())
    {
        debug("Serial port {PORT} not created yet, deferring {NAME}", "PORT",
              config->serialPort, "NAME", config->name);
        pendingDevices[config->serialPort].push_back(
            {objectPath, interfaceName});
        co_return;
    }

    addInventoryDevice(*config, *(portIter->second));
}

auto DeviceManager::addInventoryDevice(const DeviceFactoryConfigIntf& config,
                                       PortIntf::BasePort& port) -> void
{
    auto callback = [this, config](bool success) -> sdbusplus::async::task<> {
        if (success)
        {
            co_await processDeviceAdded(config);
        }
        else
        {
            auto deviceKey = config.name + "/" + config.type;
            auto iter = devices.find(deviceKey);
            if (iter != devices.end())
            {
                iter->second->requestStop();
                debug("Requested sensor device {NAME} to stop", "NAME",
                      config.name);
            }
        }
        handleSiblingProbes(config, success);
    };

    try
    {
        auto inventoryDevice = std::make_unique<InventoryIntf::Device>(
            ctx, config, port, allowedDevices, std::move(callback));
        ctx.spawn(inventoryDevice->startProbing());
        inventoryDevices[config.name][config.type] = std::move(inventoryDevice);
    }
    catch (const std::exception& e)
    {
        error("Failed to create Inventory Device for {NAME} with {ERROR}",
              "NAME", config.name, "ERROR", e);
    }
}

auto DeviceManager::handleSiblingProbes(const DeviceFactoryConfigIntf& config,
                                        bool success) -> void
{
    for (auto& [type, inv] : inventoryDevices[config.name])
    {
        if (type == config.type)
        {
            continue;
        }
        if (success && !inv->isStopped())
        {
            // Stop probing for other vendors with the same device
            // name and type (e.g., stop Delta when Artesyn probed).
            inv->requestStop(true);
        }
        else if (!success && inv->isStopped())
        {
            // Restart probing for sibling variants so a replacement
            // device of either type can be rediscovered.
            inv->restart();
            ctx.spawn(inv->startProbing());
        }
    }
}

auto DeviceManager::requestInventoryProbe(const std::string& name,
                                          const std::string& type) -> void
{
    auto nameIter = inventoryDevices.find(name);
    if (nameIter == inventoryDevices.end())
    {
        return;
    }
    auto typeIter = nameIter->second.find(type);
    if (typeIter != nameIter->second.end())
    {
        typeIter->second->requestProbe();
    }
}

auto DeviceManager::processDeviceAdded(const DeviceFactoryConfigIntf& config)
    -> sdbusplus::async::task<>
{
    auto deviceKey = config.name + "/" + config.type;
    auto iter = devices.find(deviceKey);
    if (iter != devices.end())
    {
        if (iter->second->isStopped())
        {
            info("Restarting sensor device {NAME}", "NAME", config.name);
            iter->second->restart();
            ctx.spawn(iter->second->pollRegisters());
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
        // Bind the probe helper as BaseDevice's all-reads-failed callback.
        auto device = DeviceFactoryIntf::create(
            ctx, config, *(portIter->second), events,
            std::bind_front(&DeviceManager::requestInventoryProbe, this,
                            config.name, config.type));
        ctx.spawn(device->pollRegisters());
        devices[deviceKey] = std::move(device);
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

        for (auto& [name, variants] : inventoryDevices)
        {
            std::erase_if(variants, [](const auto& entry) {
                if (entry.second->isStopped() &&
                    !entry.second->isStoppedBySibling())
                {
                    info("Removing stopped inventory device {NAME}", "NAME",
                         entry.first);
                    return true;
                }
                return false;
            });
        }
        std::erase_if(inventoryDevices,
                      [](const auto& entry) { return entry.second.empty(); });
    }
}

auto DeviceManager::processConfigRemoved(
    const sdbusplus::object_path& objectPath, const std::string& interfaceName)
    -> sdbusplus::async::task<>
{
    auto name = std::string(objectPath.filename());
    auto type = interfaceName.substr(interfaceName.rfind('.') + 1);
    info("Config removed for device {NAME}", "NAME", name);

    // Stop inventory device — its probing coroutine will clean up the
    // inventory D-Bus object and stop the sensor device via the probe
    // callback. Both are cleaned up by the regular cleanup loop.
    auto nameIter = inventoryDevices.find(name);
    if (nameIter != inventoryDevices.end())
    {
        auto variantIter = nameIter->second.find(type);
        if (variantIter != nameIter->second.end())
        {
            variantIter->second->requestStop();
        }
    }
    co_return;
}

} // namespace phosphor::modbus::rtu

auto main() -> int
{
    constexpr auto serviceName = "xyz.openbmc_project.ModbusRTU";
    sdbusplus::async::context ctx;
    using SensorIntf = sdbusplus::client::xyz::openbmc_project::sensor::Value<>;
    using InventoryIntf =
        sdbusplus::client::xyz::openbmc_project::inventory::Item<>;
    using MetricIntf = sdbusplus::client::xyz::openbmc_project::metric::Value<>;
    using SoftwareIntf =
        sdbusplus::client::xyz::openbmc_project::software::Version<>;
    sdbusplus::server::manager_t sensorManager{
        ctx, SensorIntf::namespace_path::value};
    sdbusplus::server::manager_t metricManager{
        ctx, MetricIntf::namespace_path::value};
    sdbusplus::server::manager_t inventoryManager{
        ctx, InventoryIntf::namespace_path};
    sdbusplus::server::manager_t softwareManager{ctx,
                                                 SoftwareIntf::namespace_path};

    info("Creating Modbus device manager");
    phosphor::modbus::rtu::DeviceManager deviceManager{ctx};

    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
