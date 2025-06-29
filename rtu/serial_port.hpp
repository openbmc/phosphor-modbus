#pragma once

#include "modbus/modbus.hpp"

#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Configuration/USBPort/client.hpp>

namespace phosphor::modbus::rtu
{

using USBPortConfigIntf =
    sdbusplus::client::xyz::openbmc_project::configuration::USBPort<>;

namespace serial_port
{

using ModbusIntf = phosphor::modbus::rtu::Modbus;

namespace config
{

enum class PortMode
{
    rs232,
    rs485,
    unknown
};

enum class PortType
{
    usb,
    unknown
};

struct Config
{
    std::string name = "unknown";
    PortType portType = PortType::unknown;
    std::string address = "unknown";
    uint16_t port = 0;
    uint16_t interface = 0;
    PortMode portMode = PortMode::unknown;
    uint32_t baudRate = 0;
    uint16_t rtsDelay = 0;
};

auto getConfig(sdbusplus::async::context& ctx,
               sdbusplus::message::object_path objectPath)
    -> sdbusplus::async::task<std::optional<Config>>;

} // namespace config

class SerialPort
{
  public:
    explicit SerialPort(sdbusplus::async::context& ctx, config::Config& config);

    auto readHoldingRegisters(uint8_t deviceAddress, uint16_t registerOffset,
                              uint32_t baudRate, Parity parity,
                              std::vector<uint16_t>& registers)
        -> sdbusplus::async::task<bool>;

  private:
    sdbusplus::async::context& ctx;
    config::Config config;
    int fd = -1;
    std::unique_ptr<ModbusIntf> modbus;
    // TODO: Add a async mutex to protect the serial port
};

} // namespace serial_port

} // namespace phosphor::modbus::rtu
