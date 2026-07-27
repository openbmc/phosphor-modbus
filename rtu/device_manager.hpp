#pragma once

#include "common/entity_manager_interface.hpp"
#include "common/events.hpp"
#include "config/allowed_devices.hpp"
#include "device/base_device.hpp"
#include "device/device_factory.hpp"
#include "inventory/modbus_inventory.hpp"
#include "port/base_port.hpp"

#include <sdbusplus/async.hpp>

namespace phosphor::modbus::rtu
{

namespace InventoryIntf = phosphor::modbus::rtu::inventory;
namespace PortIntf = phosphor::modbus::rtu::port;
namespace ModbusIntf = phosphor::modbus::rtu;
namespace DeviceIntf = phosphor::modbus::rtu::device;
namespace EventIntf = phosphor::modbus::events;

class DeviceManager
{
  public:
    DeviceManager() = delete;
    DeviceManager(const DeviceManager&) = delete;
    DeviceManager& operator=(const DeviceManager&) = delete;
    DeviceManager(DeviceManager&&) = delete;
    DeviceManager& operator=(DeviceManager&&) = delete;

    explicit DeviceManager(sdbusplus::async::context& ctx);

  private:
    // Maps device type (e.g. "Artesyn7000552531000PowerShelf") to its
    // inventory device. Multiple variants can exist for the same physical slot.
    using inventory_variant_map_t =
        std::unordered_map<std::string, std::unique_ptr<InventoryIntf::Device>>;
    // Maps device name (e.g. "PSU_SHELF_1") to its variants.
    using inventory_device_map_t =
        std::unordered_map<std::string, inventory_variant_map_t>;

    using port_map_t =
        std::unordered_map<std::string, std::unique_ptr<PortIntf::BasePort>>;

    using device_map_t =
        std::unordered_map<std::string,
                           std::unique_ptr<DeviceIntf::BaseDevice>>;

    // Identity of a device config waiting for its serial port.
    struct PendingDevice
    {
        sdbusplus::object_path objectPath;
        std::string interfaceName;
    };

    /** @brief Create all ports, then all device configs, so each device binds
     *  to its port on the first pass without re-scanning per port addition. */
    auto discoverConfigs() -> sdbusplus::async::task<>;

    auto processConfigAdded(const sdbusplus::object_path& objectPath,
                            const std::string& interfaceName,
                            const entity_manager::ConfigData& inInterfaces)
        -> sdbusplus::async::task<>;

    auto processPortAdded(const sdbusplus::object_path& objectPath,
                          const std::string& interfaceName)
        -> sdbusplus::async::task<>;

    auto processInventoryAdded(const sdbusplus::object_path& objectPath,
                               const std::string& interfaceName,
                               const entity_manager::ConfigData& inInterfaces)
        -> sdbusplus::async::task<>;

    /** @brief Create and start the inventory device for a config on a port. */
    auto addInventoryDevice(const device::config::DeviceFactoryConfig& config,
                            PortIntf::BasePort& port) -> void;

    auto processDeviceAdded(const device::config::DeviceFactoryConfig& config)
        -> sdbusplus::async::task<>;

    /** @brief Stop sibling inventory variants or restart them based on
     *         probe result. */
    auto handleSiblingProbes(const device::config::DeviceFactoryConfig& config,
                             bool success) -> void;

    auto processConfigRemoved(const sdbusplus::object_path& objectPath,
                              const std::string& interfaceName)
        -> sdbusplus::async::task<>;

    /** @brief Periodically removes sensor devices whose polling coroutine
     *         has stopped after a stop request.
     */
    auto cleanupStoppedDevices() -> sdbusplus::async::task<>;

    sdbusplus::async::context& ctx;
    entity_manager::EntityManagerInterface entityManager;
    EventIntf::Events events;
    config::AllowedDevices allowedDevices;
    inventory_device_map_t inventoryDevices;
    port_map_t ports;
    device_map_t devices; // Modbus devices
    // Device configs waiting for their serial port, keyed by serial port name.
    // Drained when the matching port is created.
    std::unordered_map<std::string, std::vector<PendingDevice>> pendingDevices;
};

} // namespace phosphor::modbus::rtu
