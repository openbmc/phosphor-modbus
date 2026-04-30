#pragma once

#include "base_config.hpp"
#include "common/register_span.hpp"
#include "port/base_port.hpp"

#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Decorator/Asset/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/Chassis/aserver.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace phosphor::modbus::rtu::inventory
{

namespace ModbusIntf = phosphor::modbus::rtu;
using SerialPortIntf = phosphor::modbus::rtu::port::BasePort;

// Forward declaration for InventoryServerType alias
class InventoryServer;

using InventoryServerType = sdbusplus::async::server_t<
    InventoryServer,
    sdbusplus::aserver::xyz::openbmc_project::inventory::item::details::Chassis,
    sdbusplus::aserver::xyz::openbmc_project::inventory::decorator::details::
        Asset,
    sdbusplus::aserver::xyz::openbmc_project::association::details::
        Definitions>;

class InventoryServer : public InventoryServerType
{
  public:
    using ChassisIntf = sdbusplus::aserver::xyz::openbmc_project::inventory::
        item::details::Chassis<InventoryServer, InventoryServerType>;
    using AssetIntf = sdbusplus::aserver::xyz::openbmc_project::inventory::
        decorator::details::Asset<InventoryServer, InventoryServerType>;
    using AssocIntf = sdbusplus::aserver::xyz::openbmc_project::association::
        details::Definitions<InventoryServer, InventoryServerType>;

    InventoryServer(sdbusplus::async::context& ctx, const char* path,
                    ChassisIntf::properties_t chassisProps,
                    AssetIntf::properties_t assetProps,
                    AssocIntf::properties_t assocProps) :
        InventoryServerType(ctx, path, chassisProps, assetProps, assocProps)
    {}

    void emit_added()
    {
        ChassisIntf::emit_added();
        AssetIntf::emit_added();
        AssocIntf::emit_added();
    }

    void emit_removed()
    {
        ChassisIntf::emit_removed();
        AssetIntf::emit_removed();
        AssocIntf::emit_removed();
    }
};

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
     *         inventory object on D-Bus and invokes the probe callback.
     *         On failure of a previously discovered device, removes the
     *         inventory object and invokes the callback. Undiscovered devices
     *         are marked dormant on failure. */
    auto probeDevice() -> sdbusplus::async::task<void>;

    auto addInventoryServer() -> sdbusplus::async::task<void>;

    auto isDormant() const -> bool
    {
        return dormant;
    }

    /** @brief Request the probing coroutine to stop. */
    auto requestStop() -> void
    {
        stopRequested = true;
    }

    /** @brief Returns true after the probing coroutine has exited. */
    auto isStopped() const -> bool
    {
        return stopped;
    }

    static constexpr auto inventoryServerPath =
        "/xyz/openbmc_project/inventory/system/chassis";

    const config::Config config;

  private:
    /** @brief Handles a failed probe read — removes the inventory object
     *         for a previously discovered device, or marks an undiscovered
     *         device as dormant. */
    auto handleProbeFailed() -> sdbusplus::async::task<void>;

    auto isRunning() const -> bool;

    /** @brief Returns true if the device is still within its dormant period
     *         and should be skipped. Returns false if the device is not dormant
     *         or its dormant period has expired, in which case the dormant
     *         state is also cleared. */
    auto checkAndClearDormant() -> bool;

    sdbusplus::async::context& ctx;
    SerialPortIntf& port;
    ProbeCallback probeCallback;
    std::unique_ptr<InventoryServer> inventoryServer;
    /** @brief Duration to skip probing after failure */
    std::chrono::seconds dormantPeriod;
    /** @brief Whether the device is currently dormant */
    bool dormant = false;
    /** @brief When dormant state began */
    std::chrono::steady_clock::time_point dormantSince;
    /** @brief Whether the probe value mismatch warning has been logged */
    bool mismatchLogged = false;
    bool stopRequested = false;
    bool stopped = false;
    std::vector<RegisterSpan> registerSpans;
    std::vector<uint16_t> readBuffer;
};

} // namespace phosphor::modbus::rtu::inventory
