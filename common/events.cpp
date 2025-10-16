#include "events.hpp"

#include <phosphor-logging/commit.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/event.hpp>
#include <xyz/openbmc_project/Sensor/event.hpp>
#include <xyz/openbmc_project/State/Fan/event.hpp>
#include <xyz/openbmc_project/State/Filter/event.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/event.hpp>
#include <xyz/openbmc_project/State/Power/event.hpp>
#include <xyz/openbmc_project/State/Pump/event.hpp>
#include <xyz/openbmc_project/State/SMC/event.hpp>

namespace phosphor::modbus::events
{

PHOSPHOR_LOG2_USING;

const std::unordered_map<EventLevel, std::string> eventLevelToName = {
    {EventLevel::critical, "Critical"},
    {EventLevel::warning, "Warning"},
};

auto Events::generateSensorReadingEvent(
    sdbusplus::message::object_path objectPath, EventLevel level, double value,
    SensorValueIntf::Unit unit, bool asserted) -> sdbusplus::async::task<>
{
    namespace error_intf =
        sdbusplus::error::xyz::openbmc_project::sensor::Threshold;
    namespace event_intf =
        sdbusplus::event::xyz::openbmc_project::sensor::Threshold;

    auto eventName =
        objectPath.str + ".threshold." + eventLevelToName.at(level);
    auto pendingEvent = pendingEvents.find(eventName);

    if (asserted)
    {
        if (pendingEvent == pendingEvents.end())
        {
            sdbusplus::message::object_path eventPath{};
            if (level == EventLevel::critical)
            {
                eventPath = co_await lg2::commit(
                    ctx, error_intf::ReadingCritical("SENSOR_NAME", objectPath,
                                                     "READING_VALUE", value,
                                                     "UNITS", unit));
            }
            else
            {
                eventPath = co_await lg2::commit(
                    ctx, error_intf::ReadingWarning("SENSOR_NAME", objectPath,
                                                    "READING_VALUE", value,
                                                    "UNITS", unit));
            }

            pendingEvents.emplace(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(ctx,
                                 event_intf::SensorReadingNormalRange(
                                     "SENSOR_NAME", objectPath, "READING_VALUE",
                                     value, "UNITS", unit));

            pendingEvents.erase(eventName);
        }
    }

    debug("Sensor reading event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generateSensorFailureEvent(
    sdbusplus::message::object_path objectPath, bool asserted)
    -> sdbusplus::async::task<>
{
    namespace error_intf = sdbusplus::error::xyz::openbmc_project::Sensor;
    namespace event_intf = sdbusplus::event::xyz::openbmc_project::Sensor;

    auto eventName = objectPath.str + ".SensorFailure";
    auto pendingEvent = pendingEvents.find(eventName);

    if (asserted)
    {
        if (pendingEvent == pendingEvents.end())
        {
            auto eventPath = co_await lg2::commit(
                ctx, error_intf::SensorFailure("SENSOR_NAME", objectPath));
            pendingEvents.emplace(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(
                ctx, event_intf::SensorRestored("SENSOR_NAME", objectPath));
            pendingEvents.erase(eventName);
        }
    }

    debug("Sensor failure event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generateControllerFailureEvent(
    sdbusplus::message::object_path objectPath, std::string additionalInfo,
    bool asserted) -> sdbusplus::async::task<>
{
    namespace error_intf = sdbusplus::error::xyz::openbmc_project::state::SMC;
    namespace event_intf = sdbusplus::event::xyz::openbmc_project::state::SMC;

    auto eventName = objectPath.str + ".ControllerFailure." + additionalInfo;
    auto pendingEvent = pendingEvents.find(eventName);

    if (asserted)
    {
        if (pendingEvent == pendingEvents.end())
        {
            auto eventPath = co_await lg2::commit(
                ctx, error_intf::SMCFailed("IDENTIFIER", objectPath,
                                           "FAILURE_TYPE", additionalInfo));
            pendingEvents.emplace(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(
                ctx, event_intf::SMCRestored("IDENTIFIER", objectPath));

            pendingEvents.erase(eventName);
        }
    }

    debug("Controller failure event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generatePowerFaultEvent(sdbusplus::message::object_path objectPath,
                                     std::string additionalInfo, bool asserted)
    -> sdbusplus::async::task<>
{
    namespace error_intf = sdbusplus::error::xyz::openbmc_project::state::Power;
    namespace event_intf = sdbusplus::event::xyz::openbmc_project::state::Power;

    auto eventName = objectPath.str + ".PowerFailure." + additionalInfo;
    auto pendingEvent = pendingEvents.find(eventName);

    if (asserted)
    {
        if (pendingEvent == pendingEvents.end())
        {
            auto eventPath = co_await lg2::commit(
                ctx,
                error_intf::PowerRailFault("POWER_RAIL", objectPath,
                                           "FAILURE_DATA", additionalInfo));
            pendingEvents.emplace(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(ctx, event_intf::PowerRailFaultRecovered(
                                          "POWER_RAIL", objectPath));

            pendingEvents.erase(eventName);
        }
    }

    debug("Power fault event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generateFilterFailureEvent(
    sdbusplus::message::object_path objectPath, bool asserted)
    -> sdbusplus::async::task<>
{
    namespace error_intf =
        sdbusplus::error::xyz::openbmc_project::state::Filter;
    namespace event_intf =
        sdbusplus::event::xyz::openbmc_project::state::Filter;

    auto eventName = objectPath.str + ".FilterFailure";
    auto pendingEvent = pendingEvents.find(eventName);

    if (asserted)
    {
        if (pendingEvent == pendingEvents.end())
        {
            auto eventPath = co_await lg2::commit(
                ctx,
                error_intf::FilterRequiresService("FILTER_NAME", objectPath));
            pendingEvents.emplace(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(
                ctx, event_intf::FilterRestored("FILTER_NAME", objectPath));

            pendingEvents.erase(eventName);
        }
    }

    debug("Filter failure event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generatePumpFailureEvent(
    sdbusplus::message::object_path objectPath, bool asserted)
    -> sdbusplus::async::task<>
{
    namespace error_intf = sdbusplus::error::xyz::openbmc_project::state::Pump;
    namespace event_intf = sdbusplus::event::xyz::openbmc_project::state::Pump;

    auto eventName = objectPath.str + ".PumpFailure";
    auto pendingEvent = pendingEvents.find(eventName);

    if (asserted)
    {
        if (pendingEvent == pendingEvents.end())
        {
            auto eventPath = co_await lg2::commit(
                ctx, error_intf::PumpFailed("PUMP_NAME", objectPath));
            pendingEvents.emplace(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(
                ctx, event_intf::PumpRestored("PUMP_NAME", objectPath));

            pendingEvents.erase(eventName);
        }
    }

    debug("Pump failure event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generateFanFailureEvent(sdbusplus::message::object_path objectPath,
                                     bool asserted) -> sdbusplus::async::task<>
{
    namespace error_intf = sdbusplus::error::xyz::openbmc_project::state::Fan;
    namespace event_intf = sdbusplus::event::xyz::openbmc_project::state::Fan;

    auto eventName = objectPath.str + ".FanFailure";
    auto pendingEvent = pendingEvents.find(eventName);

    if (asserted)
    {
        if (pendingEvent == pendingEvents.end())
        {
            auto eventPath = co_await lg2::commit(
                ctx, error_intf::FanFailed("FAN_NAME", objectPath));
            pendingEvents.emplace(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(
                ctx, event_intf::FanRestored("FAN_NAME", objectPath));

            pendingEvents.erase(eventName);
        }
    }

    debug("Fan failure event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generateLeakDetectedEvent(
    sdbusplus::message::object_path objectPath, EventLevel level, bool asserted)
    -> sdbusplus::async::task<>
{
    auto eventName = objectPath.str + ".Leak." + eventLevelToName.at(level);

    if (!asserted)
    {
        auto pendingEvent = pendingEvents.find(eventName);
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            using DetectorNormal = sdbusplus::event::xyz::openbmc_project::
                state::leak::Detector::LeakDetectedNormal;
            co_await lg2::commit(ctx,
                                 DetectorNormal("DETECTOR_NAME", objectPath));

            pendingEvents.erase(eventName);
        }
        co_return;
    }

    namespace error_intf =
        sdbusplus::error::xyz::openbmc_project::state::leak::Detector;
    sdbusplus::message::object_path eventPath{};

    if (level == EventLevel::critical)
    {
        eventPath = co_await lg2::commit(
            ctx, error_intf::LeakDetectedCritical("DETECTOR_NAME", objectPath));
        error("Critical leak detected for {PATH}", "PATH", objectPath);
    }
    else
    {
        eventPath = co_await lg2::commit(
            ctx, error_intf::LeakDetectedWarning("DETECTOR_NAME", objectPath));
        warning("Warning leak detected for {PATH}", "PATH", objectPath);
    }
    pendingEvents[eventName] = eventPath;
}

} // namespace phosphor::modbus::events
