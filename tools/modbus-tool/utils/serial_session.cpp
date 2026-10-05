#include "utils/serial_session.hpp"

#include "inventory/inventory_utils.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <phosphor-logging/lg2.hpp>

#include <stdexcept>
#include <utility>

namespace modbus_tool
{

namespace ModbusIntf = phosphor::modbus::rtu;
namespace InventoryIntf = phosphor::modbus::rtu::inventory;

SerialSession::SerialSession(
    sdbusplus::async::context& ctx,
    const PortIntf::config::PortFactoryConfig& portConfig,
    const std::string& devicePath) : portConfig(portConfig)
{
    fd = open(devicePath.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        throw std::runtime_error("Failed to open serial port " + devicePath);
    }

    try
    {
        modbus = std::make_unique<ModbusIntf::Modbus>(
            ctx, fd, portConfig.baudRate, portConfig.rtsDelay,
            portConfig.timeout);
    }
    catch (...)
    {
        close(fd);
        fd = -1;
        throw;
    }
}

SerialSession::~SerialSession()
{
    modbus.reset();
    if (fd >= 0)
    {
        close(fd);
    }
}

auto SerialSession::applyProfile(const ConfigIntf::Config& config) -> bool
{
    return modbus->setProperties(portConfig.baudRate, config.profile.parity);
}

auto SerialSession::probe(const ConfigIntf::Config& config, bool& matched)
    -> sdbusplus::async::task<std::vector<uint16_t>>
{
    const auto& probeRegister = config.profile.probeRegister;
    std::vector<uint16_t> registers(probeRegister.size);

    if (!applyProfile(config) ||
        !co_await modbus->readHoldingRegisters(config.address,
                                               probeRegister.offset, registers))
    {
        co_return std::vector<uint16_t>{};
    }

    matched = InventoryIntf::matchesProbeValue(registers, probeRegister);
    co_return registers;
}

auto SerialSession::readHoldingRegisters(
    uint8_t deviceAddress, uint16_t registerOffset,
    std::span<uint16_t> registers) -> sdbusplus::async::task<bool>
{
    co_return co_await modbus->readHoldingRegisters(deviceAddress,
                                                    registerOffset, registers);
}

auto SerialSession::readFileRecord(uint8_t deviceAddress,
                                   std::span<ModbusIntf::FileRecord> records)
    -> sdbusplus::async::task<bool>
{
    co_return co_await modbus->readFileRecord(deviceAddress, records);
}

auto SerialSession::writeSingleRegister(uint8_t deviceAddress,
                                        uint16_t registerOffset, uint16_t value)
    -> sdbusplus::async::task<bool>
{
    co_return co_await modbus->writeSingleRegister(deviceAddress,
                                                   registerOffset, value);
}

auto SerialSession::writeMultipleRegisters(
    uint8_t deviceAddress, uint16_t registerOffset,
    std::span<const uint16_t> registers) -> sdbusplus::async::task<bool>
{
    co_return co_await modbus->writeMultipleRegisters(
        deviceAddress, registerOffset, registers);
}

} // namespace modbus_tool
