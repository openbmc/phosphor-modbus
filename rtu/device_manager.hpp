#pragma once

#include "common/entity_manager_interface.hpp"
#include "common/events.hpp"
#include "device.hpp"
#include "inventory_device.hpp"
#include "serial_port.hpp"

#include <sdbusplus/async.hpp>

namespace phosphor::modbus::rtu
{

namespace InventoryIntf = phosphor::modbus::rtu::inventory;
namespace SerialPortIntf = phosphor::modbus::rtu::serial_port;
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
    using inventory_device_map_t =
        std::unordered_map<std::string, std::unique_ptr<InventoryIntf::Device>>;
    using serial_port_map_t =
        std::unordered_map<std::string,
                           std::unique_ptr<SerialPortIntf::SerialPort>>;
    using device_info_t =
        std::pair<ModbusIntf::Parity, uint32_t>; // < Parity, BaudRate >

    using device_map_t =
        std::unordered_map<std::string, std::unique_ptr<DeviceIntf::Device>>;

    auto processConfigAdded(const sdbusplus::message::object_path& objectPath,
                            const std::string& interfaceName)
        -> sdbusplus::async::task<>;

    auto processPortAdded(const sdbusplus::message::object_path& objectPath)
        -> sdbusplus::async::task<>;

    auto processInventoryAdded(
        const sdbusplus::message::object_path& objectPath)
        -> sdbusplus::async::task<>;

    auto processDeviceAdded(const sdbusplus::message::object_path& objectPath,
                            const std::string& interfaceName)
        -> sdbusplus::async::task<>;

    auto processConfigRemoved(const sdbusplus::message::object_path& objectPath,
                              const std::string& interfaceName)
        -> sdbusplus::async::task<>;

    auto getDeviceInfoFromInventory(const std::string& deviceName,
                                    uint8_t address, std::string& portName)
        -> std::optional<device_info_t>;

    sdbusplus::async::context& ctx;
    entity_manager::EntityManagerInterface entityManager;
    EventIntf::Events events;
    inventory_device_map_t inventoryDevices;
    serial_port_map_t serialPorts;
    device_map_t devices; // Modbus devices
};

} // namespace phosphor::modbus::rtu
