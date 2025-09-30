#pragma once

#include "modbus/modbus.hpp"
#include "port/base_port.hpp"

#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Inventory/Source/Modbus/FRU/aserver.hpp>

#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace phosphor::modbus::rtu::inventory
{

class Device;

namespace ModbusIntf = phosphor::modbus::rtu;
using SerialPortIntf = phosphor::modbus::rtu::port::BasePort;
using InventorySourceIntf =
    sdbusplus::aserver::xyz::openbmc_project::inventory::source::modbus::FRU<
        Device>;

namespace config
{

struct Register
{
    std::string name = "unknown";
    uint16_t offset = 0;
    uint8_t size = 0;
};

struct AddressRange
{
    uint8_t start;
    uint8_t end;
};

struct Config
{
    using address_range_arr_t = std::vector<AddressRange>;
    using port_address_map_t =
        std::map<std::string,
                 address_range_arr_t>; // <port name, device address range list>

    std::string name = "unknown";
    port_address_map_t addressMap = {};
    std::vector<Register> registers = {};
    ModbusIntf::Parity parity = ModbusIntf::Parity::unknown;
    uint32_t baudRate = 0;
};

auto getConfig(sdbusplus::async::context& ctx,
               sdbusplus::message::object_path objectPath)
    -> sdbusplus::async::task<std::optional<Config>>;

} // namespace config

class Device
{
  public:
    Device() = delete;
    using serial_port_map_t =
        std::unordered_map<std::string, std::unique_ptr<SerialPortIntf>>;

    explicit Device(sdbusplus::async::context& ctx,
                    const config::Config& config,
                    serial_port_map_t& serialPorts);

    auto probePorts() -> sdbusplus::async::task<void>;

    auto probePort(std::string portName) -> sdbusplus::async::task<void>;

    auto probeDevice(uint8_t address, const std::string& portName,
                     SerialPortIntf& port) -> sdbusplus::async::task<void>;

    auto addInventorySource(uint8_t address, const std::string& portName,
                            SerialPortIntf& port)
        -> sdbusplus::async::task<void>;

  private:
    sdbusplus::async::context& ctx;
    const config::Config config;
    serial_port_map_t& serialPorts;
    std::map<std::string, std::unique_ptr<InventorySourceIntf>>
        inventorySources;
};

} // namespace phosphor::modbus::rtu::inventory
