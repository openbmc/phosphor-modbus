#pragma once

#include "base_config.hpp"
#include "common/events.hpp"
#include "common/register_span.hpp"
#include "firmware/device_firmware.hpp"
#include "port/base_port.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Critical/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Warning/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Value/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Value/client.hpp>
#include <xyz/openbmc_project/State/Decorator/Availability/aserver.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/aserver.hpp>

#include <cstdint>
#include <span>

namespace phosphor::modbus::rtu::device
{

namespace ProfileIntf = phosphor::modbus::rtu::profile;
using SensorValueIntf =
    sdbusplus::client::xyz::openbmc_project::sensor::Value<>;

/** @brief Returns the D-Bus object path suffix for a sensor type.
 *  @throws std::invalid_argument if type is unknown. */
auto getPathSuffix(ProfileIntf::SensorType type) -> std::string_view;

/** @brief Returns the sensor unit corresponding to a sensor type.
 *  @throws std::invalid_argument if type is unknown. */
auto getUnit(ProfileIntf::SensorType type) -> SensorValueIntf::Unit;

class BaseDevice;

using SensorIntf = sdbusplus::async::server_t<
    BaseDevice, sdbusplus::aserver::xyz::openbmc_project::sensor::Value,
    sdbusplus::aserver::xyz::openbmc_project::state::decorator::Availability,
    sdbusplus::aserver::xyz::openbmc_project::state::decorator::
        OperationalStatus,
    sdbusplus::aserver::xyz::openbmc_project::sensor::threshold::Warning,
    sdbusplus::aserver::xyz::openbmc_project::sensor::threshold::Critical,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

using PortIntf = phosphor::modbus::rtu::port::BasePort;
namespace EventIntf = phosphor::modbus::events;

class BaseDevice
{
  public:
    BaseDevice() = delete;

    explicit BaseDevice(sdbusplus::async::context& ctx,
                        const config::Config& config, PortIntf& serialPort,
                        EventIntf::Events& events);

    /** @brief Poll sensor and status registers in a timed loop. */
    auto pollRegisters() -> sdbusplus::async::task<void>;

    /** @brief Request the sensor polling coroutine to stop. */
    auto stop() -> void
    {
        stopped = true;
    }

    /** @brief Returns true after the sensor polling coroutine has exited. */
    auto isFinished() const -> bool
    {
        return finished;
    }

    /** @brief Reset stop/finished flags so the polling loop can be
     *         restarted. */
    auto restart() -> void
    {
        stopped = false;
        finished = false;
    }

  private:
    // Pre-computed pairing of register config and its corresponding sensor,
    // built at construction to avoid map lookups on every poll cycle.
    struct SensorEntry
    {
        const ProfileIntf::SensorRegister& reg;
        SensorIntf& sensor;
    };

    struct PollBucket
    {
        std::chrono::seconds pollInterval;
        std::vector<RegisterSpan> spans;
        std::chrono::steady_clock::time_point nextPollTime;
    };

    // Pre-computed pairing of status register address and its status bits,
    // built at construction for span-based batch reads.
    struct StatusEntry
    {
        uint16_t address;
        const std::vector<ProfileIntf::StatusBit>* statusBits;
    };

    /** @brief Create D-Bus sensor objects from the device profile. */
    auto createSensors() -> void;

    /** @brief Build poll buckets with merged sensor and status spans. */
    auto buildPollBuckets() -> void;

    /** @brief Group sensor indices by poll interval, appending status
     *         register indices to the default poll interval bucket. */
    auto groupByPollInterval()
        -> std::vector<std::pair<std::chrono::seconds, std::vector<size_t>>>;

    /** @brief Read and process all spans in a single poll bucket. */
    auto pollBucket(PollBucket& bucket) -> sdbusplus::async::task<void>;

    /** @brief Convert raw register data and update a sensor value. */
    auto processSensorEntry(size_t sensorIdx,
                            std::span<const uint16_t> spanBuffer,
                            uint16_t spanStartOffset) -> void;

    /** @brief Process status bits for a single status register entry. */
    auto processStatusEntry(size_t statusIdx,
                            std::span<const uint16_t> spanBuffer,
                            uint16_t spanStartOffset)
        -> sdbusplus::async::task<void>;

    /** @brief Generate a Redfish event for a status bit assertion. */
    auto generateEvent(const ProfileIntf::StatusBit& statusBit,
                       const sdbusplus::object_path& objectPath,
                       double sensorValue, SensorIntf::Unit sensorUnit,
                       bool statusAsserted) -> sdbusplus::async::task<void>;

    using sensors_map_t =
        std::unordered_map<std::string, std::unique_ptr<SensorIntf>>;
    sdbusplus::async::context& ctx;
    const config::Config config;
    PortIntf& serialPort;
    EventIntf::Events& events;
    std::unique_ptr<DeviceFirmware> currentFirmware;
    sensors_map_t sensors;
    std::vector<SensorEntry> sensorEntries;
    std::vector<PollBucket> pollBuckets;
    std::vector<StatusEntry> statusEntries;
    std::vector<uint16_t> readBuffer;
    bool stopped = false;
    bool finished = false;
};

} // namespace phosphor::modbus::rtu::device
