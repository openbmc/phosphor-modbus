#pragma once

#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Sensor/Value/client.hpp>

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

    explicit Events(sdbusplus::async::context& ctx) : ctx(ctx) {}

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

  private:
    /** @brief Map type for event name to log event object path */
    using event_map_t = std::map<std::string, sdbusplus::object_path>;

    sdbusplus::async::context& ctx;
    event_map_t pendingEvents;
};

} // namespace phosphor::modbus::events
