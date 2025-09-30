#pragma once

#include "common/entity_manager_interface.hpp"
#include "inventory_device.hpp"
#include "port/base_port.hpp"

#include <sdbusplus/async.hpp>

namespace phosphor::modbus::rtu
{

namespace InventoryIntf = phosphor::modbus::rtu::inventory;
namespace PortIntf = phosphor::modbus::rtu::port;

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
    using inventory_device_map_t =
        std::unordered_map<std::string, std::unique_ptr<InventoryIntf::Device>>;
    using port_map_t =
        std::unordered_map<std::string, std::unique_ptr<PortIntf::BasePort>>;

    auto processConfigAdded(const sdbusplus::message::object_path& objectPath,
                            const std::string& interfaceName)
        -> sdbusplus::async::task<>;

    auto processConfigRemoved(const sdbusplus::message::object_path& objectPath,
                              const std::string& interfaceName)
        -> sdbusplus::async::task<>;

    sdbusplus::async::context& ctx;
    entity_manager::EntityManagerInterface entityManager;
    inventory_device_map_t inventoryDevices;
    port_map_t ports;
};

} // namespace phosphor::modbus::rtu
