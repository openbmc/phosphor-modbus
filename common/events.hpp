#pragma once

#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Sensor/Value/client.hpp>

#include <filesystem>
#include <map>
#include <string>

namespace phosphor::modbus::events
{

using SensorValueIntf =
    sdbusplus::client::xyz::openbmc_project::sensor::Value<>;

enum class EventLevel
{
    critical,
    warning
};

class Events
{
  public:
    Events() = delete;

    explicit Events(sdbusplus::async::context& ctx,
                    std::filesystem::path stateDir);

    auto generateSensorReadingEvent(sdbusplus::object_path objectPath,
                                    EventLevel level, double value,
                                    SensorValueIntf::Unit unit, bool asserted)
        -> sdbusplus::async::task<>;

    auto generateSensorFailureEvent(sdbusplus::object_path objectPath,
                                    bool asserted) -> sdbusplus::async::task<>;

    auto generateControllerFailureEvent(sdbusplus::object_path objectPath,
                                        std::string additionalInfo,
                                        bool asserted)
        -> sdbusplus::async::task<>;

    auto generatePowerFaultEvent(sdbusplus::object_path objectPath,
                                 std::string additionalInfo, bool asserted)
        -> sdbusplus::async::task<>;

    auto generateFilterFailureEvent(sdbusplus::object_path objectPath,
                                    bool asserted) -> sdbusplus::async::task<>;

    auto generatePumpFailureEvent(sdbusplus::object_path objectPath,
                                  bool asserted) -> sdbusplus::async::task<>;

    auto generateFanFailureEvent(sdbusplus::object_path objectPath,
                                 bool asserted) -> sdbusplus::async::task<>;

    auto generateLeakDetectedEvent(sdbusplus::object_path objectPath,
                                   EventLevel level, bool asserted)
        -> sdbusplus::async::task<>;

    // Button Pressed

    /** @brief Load persisted pending events from disk. Call once at startup,
     *  before any device begins polling. */
    void restore();

  private:
    /** @brief Map type for event name to log event object path */
    using event_map_t = std::map<std::string, sdbusplus::object_path>;

    /** @brief Record a pending event and persist the map. */
    void addPending(const std::string& eventName,
                    const sdbusplus::object_path& eventPath);

    /** @brief Drop a pending event and persist the map. */
    void removePending(const std::string& eventName);

    /** @brief True if the event is tracked and its log entry still exists.
     *  A tracked entry whose log was rotated out ("wrapped") reports false so
     *  the caller regenerates it. */
    auto isPending(const std::string& eventName)
        -> sdbusplus::async::task<bool>;

    /** @brief Resolve a log entry, ignoring an already rotated-out entry. */
    auto resolvePending(const sdbusplus::object_path& eventPath)
        -> sdbusplus::async::task<void>;

    /** @brief Atomically write pendingEvents to stateFile (best-effort). */
    void persist();

    sdbusplus::async::context& ctx;
    std::filesystem::path stateFile;
    bool stateDirReady = false;
    event_map_t pendingEvents;
};

} // namespace phosphor::modbus::events
