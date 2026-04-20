#pragma once

#include "base_config.hpp"
#include "common/register_span.hpp"
#include "port/base_port.hpp"

#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Inventory/Source/Modbus/FRU/aserver.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace phosphor::modbus::rtu::inventory
{

class Device;

namespace ModbusIntf = phosphor::modbus::rtu;
using SerialPortIntf = phosphor::modbus::rtu::port::BasePort;
using InventorySourceIntf =
    sdbusplus::aserver::xyz::openbmc_project::inventory::source::modbus::FRU<
        Device>;

class Device
{
  public:
    Device() = delete;

    using ProbeCallback = std::function<sdbusplus::async::task<>(bool success)>;

    explicit Device(
        sdbusplus::async::context& ctx, const config::Config& config,
        SerialPortIntf& port, ProbeCallback probeCallback = nullptr,
        std::chrono::seconds dormantPeriod = std::chrono::seconds(0));

    /** @brief Starts continuously probing the device at a fixed interval
     *         until the context is stopped. */
    auto startProbing() -> sdbusplus::async::task<void>;

    /** @brief Performs a single probe attempt. On first success, creates the
     *         inventory source on D-Bus and invokes the probe callback.
     *         On failure of a previously discovered device, removes the
     *         inventory source and invokes the callback. Undiscovered devices
     *         are marked dormant on failure. */
    auto probeDevice() -> sdbusplus::async::task<void>;

    auto addInventorySource() -> sdbusplus::async::task<void>;

    auto isDormant() const -> bool
    {
        return dormant;
    }

    const config::Config config;

  private:
    /** @brief Returns true if the device is still within its dormant period
     *         and should be skipped. Returns false if the device is not dormant
     *         or its dormant period has expired, in which case the dormant
     *         state is also cleared. */
    auto checkAndClearDormant() -> bool;

    sdbusplus::async::context& ctx;
    SerialPortIntf& port;
    ProbeCallback probeCallback;
    std::unique_ptr<InventorySourceIntf> inventorySource;
    /** @brief Duration to skip probing after failure */
    std::chrono::seconds dormantPeriod;
    /** @brief Whether the device is currently dormant */
    bool dormant = false;
    /** @brief When dormant state began */
    std::chrono::steady_clock::time_point dormantSince;
    std::vector<RegisterSpan> registerSpans;
};

} // namespace phosphor::modbus::rtu::inventory
