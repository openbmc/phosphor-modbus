#include "base_port.hpp"

#include "common/entity_manager_interface.hpp"

#include <fcntl.h>

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Configuration/USBPort/client.hpp>

#include <filesystem>
#include <format>
#include <optional>
#include <regex>
#include <utility>

namespace phosphor::modbus::rtu::port
{

PHOSPHOR_LOG2_USING;

BasePort::BasePort(sdbusplus::async::context& ctx, const config::Config& config,
                   const std::string& devicePath) :
    PortControlIntf(ctx, sdbusplus::object_path(
                             sdbusplus::common::xyz::openbmc_project::control::
                                 Port::namespace_path) /
                             config.name),
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

    emit_added();

    info("Serial port {NAME} created successfully", "NAME", config.name);
}

BasePort::~BasePort()
{
    emit_removed();
}

auto BasePort::set_property(monitoring_enabled_t, bool enabled) -> bool
{
    if (enabled)
    {
        monitoringLock.reset();
    }
    else if (!monitoringLock)
    {
        if (auto lock = acquireExclusive())
        {
            monitoringLock.emplace(std::move(*lock));
        }
    }

    // Monitoring is enabled only when the port is not reserved, so a failed
    // reservation leaves the property unchanged.
    bool monitoringEnabled = !monitoringLock.has_value();
    bool changed = (monitoringEnabled != monitoring_enabled_);
    monitoring_enabled_ = monitoringEnabled;
    return changed;
}

ExclusiveLock::ExclusiveLock(BasePort& port) : port(&port) {}

ExclusiveLock::ExclusiveLock(ExclusiveLock&& other) noexcept : port(other.port)
{
    other.port = nullptr;
}

ExclusiveLock::~ExclusiveLock()
{
    if (port != nullptr)
    {
        port->busy = false;
    }
}

auto BasePort::acquireExclusive() -> std::optional<ExclusiveLock>
{
    // No co_await between check and set, so only one caller wins.
    if (busy)
    {
        return std::nullopt;
    }
    busy = true;
    return ExclusiveLock(*this);
}

auto BasePort::readHoldingRegisters(
    uint8_t deviceAddress, uint16_t registerOffset, uint32_t baudRate,
    Parity parity, std::span<uint16_t> registers, ExclusiveLock* lock)
    -> sdbusplus::async::task<OperationStatus>
{
    if (lock == nullptr && busy)
    {
        co_return OperationStatus::busy;
    }

    sdbusplus::async::lock_guard lg{mutex};
    co_await lg.lock();

    if (!modbus->setProperties(baudRate, parity))
    {
        error("Failed to set serial port properties");
        co_return OperationStatus::failure;
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
        co_return OperationStatus::failure;
    }

    co_return OperationStatus::success;
}

auto BasePort::writeMultipleRegisters(
    uint8_t deviceAddress, uint16_t registerOffset, uint32_t baudRate,
    Parity parity, std::span<const uint16_t> registers, ExclusiveLock* lock)
    -> sdbusplus::async::task<OperationStatus>
{
    if (lock == nullptr && busy)
    {
        co_return OperationStatus::busy;
    }

    sdbusplus::async::lock_guard lg{mutex};
    co_await lg.lock();

    if (!modbus->setProperties(baudRate, parity))
    {
        error("Failed to set serial port properties");
        co_return OperationStatus::failure;
    }

    debug(
        "Writing multiple registers to device {ADDRESS} {PORT} at offset {OFFSET}",
        "ADDRESS", lg2::hex, deviceAddress, "PORT", name, "OFFSET", lg2::hex,
        registerOffset);

    co_return co_await modbus->writeMultipleRegisters(deviceAddress,
                                                      registerOffset, registers)
        ? OperationStatus::success
        : OperationStatus::failure;
}

} // namespace phosphor::modbus::rtu::port
