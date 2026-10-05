#include "utils/serial_session.hpp"

#include "inventory/inventory_utils.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <phosphor-logging/lg2.hpp>

#include <utility>

namespace modbus_tool
{

PHOSPHOR_LOG2_USING;

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
        error("Failed to open {PATH}", "PATH", devicePath);
        return;
    }

    try
    {
        modbus = std::make_unique<ModbusIntf::Modbus>(
            ctx, fd, portConfig.baudRate, portConfig.rtsDelay,
            portConfig.timeout);
    }
    catch (const std::exception& e)
    {
        error("Failed to open {PATH}: {ERROR}", "PATH", devicePath, "ERROR", e);
        close(fd);
        fd = -1;
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

} // namespace modbus_tool
