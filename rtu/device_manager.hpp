#pragma once

#include "common/entity_manager_interface.hpp"
#include "inventory_device.hpp"
#include "serial_port.hpp"

#include <sdbusplus/async.hpp>

namespace phosphor::modbus::rtu
{

namespace InventoryIntf = phosphor::modbus::rtu::inventory;
namespace SerialPortIntf = phosphor::modbus::rtu::serial_port;

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
    using serial_port_map_t =
        std::unordered_map<std::string,
                           std::unique_ptr<SerialPortIntf::SerialPort>>;

    auto processConfigAdded(const sdbusplus::message::object_path& objectPath,
                            const std::string& interfaceName)
        -> sdbusplus::async::task<>;

    auto processConfigRemoved(const sdbusplus::message::object_path& objectPath,
                              const std::string& interfaceName)
        -> sdbusplus::async::task<>;

    sdbusplus::async::context& ctx;
    entity_manager::EntityManagerInterface entityManager;
    inventory_device_map_t inventoryDevices;
    serial_port_map_t serialPorts;
};

} // namespace phosphor::modbus::rtu
