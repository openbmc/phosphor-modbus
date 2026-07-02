#include "base_port.hpp"

#include "common/entity_manager_interface.hpp"

#include <fcntl.h>

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Configuration/USBPort/client.hpp>

#include <filesystem>
#include <format>
#include <optional>
#include <regex>

namespace phosphor::modbus::rtu::port
{

PHOSPHOR_LOG2_USING;

BasePort::BasePort(sdbusplus::async::context& ctx, const config::Config& config,
                   const std::string& devicePath) :
    name(config.name), mutex(config.name)
{
    fd = open(devicePath.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        throw std::runtime_error("Failed to open serial port " + devicePath +
                                 " with error: " + strerror(errno));
    }

    try
    {
        modbus = std::make_unique<ModbusIntf>(ctx, fd, config.baudRate,
                                              config.rtsDelay, config.timeout);
    }
    catch (...)
    {
        close(fd);
        throw std::runtime_error("Failed to create Modbus interface");
    }

    info("Serial port {NAME} created successfully", "NAME", config.name);
}

auto BasePort::readHoldingRegisters(
    uint8_t deviceAddress, uint16_t registerOffset, uint32_t baudRate,
    Parity parity, std::span<uint16_t> registers)
    -> sdbusplus::async::task<bool>
{
    sdbusplus::async::lock_guard lg{mutex};
    co_await lg.lock();

    if (!modbus->setProperties(baudRate, parity))
    {
        error("Failed to set serial port properties");
        co_return false;
    }

    debug(
        "Reading holding registers from device {ADDRESS} {PORT} at offset {OFFSET}",
        "ADDRESS", lg2::hex, deviceAddress, "PORT", name, "OFFSET", lg2::hex,
        registerOffset);

    auto ret = co_await modbus->readHoldingRegisters(deviceAddress,
                                                     registerOffset, registers);
    if (!ret)
    {
        error(
            "Failed to read holding registers from device {ADDRESS} {PORT} at offset "
            "{OFFSET}",
            "ADDRESS", lg2::hex, deviceAddress, "PORT", name, "OFFSET",
            lg2::hex, registerOffset);
    }

    co_return ret;
}

auto BasePort::writeMultipleRegisters(
    uint8_t deviceAddress, uint16_t registerOffset, uint32_t baudRate,
    Parity parity, std::span<const uint16_t> registers)
    -> sdbusplus::async::task<bool>
{
    sdbusplus::async::lock_guard lg{mutex};
    co_await lg.lock();

    if (!modbus->setProperties(baudRate, parity))
    {
        error("Failed to set serial port properties");
        co_return false;
    }

    debug(
        "Writing multiple registers to device {ADDRESS} {PORT} at offset {OFFSET}",
        "ADDRESS", lg2::hex, deviceAddress, "PORT", name, "OFFSET", lg2::hex,
        registerOffset);

    co_return co_await modbus->writeMultipleRegisters(
        deviceAddress, registerOffset, registers);
}

} // namespace phosphor::modbus::rtu::port
