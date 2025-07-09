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

const entity_manager::interface_list_t interfaces{
    USBPortConfigIntf::interface, ModbusRTUDetectIntf::interface};

DeviceManager::DeviceManager(sdbusplus::async::context& ctx) :
    ctx(ctx),
    entityManager(ctx, interfaces,
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
    if (interfaceName == ModbusRTUDetectIntf::interface &&
        serialPorts.size() == 0)
    {
        warning(
            "Skip processing ModbusRTUDetectIntf::interface as no serial ports detected yet");
        co_return;
    }

    if (interfaceName == USBPortConfigIntf::interface)
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
            error("Failed to create Serial Port for {PATH} with {ERROR}",
                  "PATH", objectPath, "ERROR", e);
            co_return;
        }
    }
    else if (interfaceName == ModbusRTUDetectIntf::interface)
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
            auto inventoryDevice = std::make_unique<InventoryIntf::Device>(
                ctx, config, serialPorts);
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
