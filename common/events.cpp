#include "events.hpp"

#include <nlohmann/json.hpp>
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

#include <filesystem>
#include <fstream>

namespace phosphor::modbus::events
{

PHOSPHOR_LOG2_USING;

namespace fs = std::filesystem;
using json = nlohmann::json;

// On-disk schema version for the pending-events state file.
static constexpr auto stateFileVersion = "1.0.0";

static constexpr auto stateFileName = "pending-events.json";

const std::unordered_map<EventLevel, std::string> eventLevelToName = {
    {EventLevel::critical, "Critical"},
    {EventLevel::warning, "Warning"},
};

Events::Events(sdbusplus::async::context& ctx, std::filesystem::path stateDir) :
    ctx(ctx), stateFile(std::move(stateDir) / stateFileName)
{}

void Events::persist()
{
    if (!stateDirReady)
    {
        std::error_code ec;
        fs::create_directories(stateFile.parent_path(), ec);
        if (ec)
        {
            error("Failed to create state directory {DIR}: {ERROR}", "DIR",
                  stateFile.parent_path().string(), "ERROR", ec.message());
            return;
        }
        stateDirReady = true;
    }

    json j;
    j["version"] = stateFileVersion;
    auto& entries = j["pendingEvents"];
    for (const auto& [name, path] : pendingEvents)
    {
        entries[name] = path.str;
    }

    auto tmp = stateFile;
    tmp += ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out)
        {
            error("Failed to open {FILE} for writing", "FILE", tmp.string());
            return;
        }
        out << j.dump(2);
    }

    std::error_code ec;
    fs::rename(tmp, stateFile, ec);
    if (ec)
    {
        error("Failed to persist pending events to {FILE}: {ERROR}", "FILE",
              stateFile.string(), "ERROR", ec.message());
    }
}

void Events::restore()
{
    pendingEvents.clear();

    std::ifstream in(stateFile);
    if (!in)
    {
        // No state file yet (first run) - nothing to restore.
        return;
    }

    json j;
    try
    {
        j = json::parse(in);
    }
    catch (const std::exception& e)
    {
        warning("Ignoring corrupt pending-events file {FILE}: {ERROR}", "FILE",
                stateFile.string(), "ERROR", e.what());
        return;
    }

    if (j.value("version", std::string{}) != stateFileVersion)
    {
        warning("Ignoring pending-events file {FILE} with unsupported version",
                "FILE", stateFile.string());
        return;
    }

    auto entries = j.value("pendingEvents", json::object());
    for (const auto& [name, path] : entries.items())
    {
        pendingEvents.emplace(name,
                              sdbusplus::object_path(path.get<std::string>()));
    }

    debug("Restored {COUNT} pending event(s) from {FILE}", "COUNT",
          pendingEvents.size(), "FILE", stateFile.string());
}

void Events::addPending(const std::string& eventName,
                        const sdbusplus::object_path& eventPath)
{
    pendingEvents.insert_or_assign(eventName, eventPath);
    persist();
}

void Events::removePending(const std::string& eventName)
{
    pendingEvents.erase(eventName);
    persist();
}

auto Events::generateSensorReadingEvent(
    sdbusplus::object_path objectPath, EventLevel level, double value,
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
            sdbusplus::object_path eventPath{};
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

            addPending(eventName, eventPath);
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

            removePending(eventName);
        }
    }

    debug("Sensor reading event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generateSensorFailureEvent(sdbusplus::object_path objectPath,
                                        bool asserted)
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
            addPending(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(
                ctx, event_intf::SensorRestored("SENSOR_NAME", objectPath));
            removePending(eventName);
        }
    }

    debug("Sensor failure event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generateControllerFailureEvent(
    sdbusplus::object_path objectPath, std::string additionalInfo,
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
            addPending(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(
                ctx, event_intf::SMCRestored("IDENTIFIER", objectPath));

            removePending(eventName);
        }
    }

    debug("Controller failure event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generatePowerFaultEvent(sdbusplus::object_path objectPath,
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
            addPending(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(ctx, event_intf::PowerRailFaultRecovered(
                                          "POWER_RAIL", objectPath));

            removePending(eventName);
        }
    }

    debug("Power fault event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generateFilterFailureEvent(sdbusplus::object_path objectPath,
                                        bool asserted)
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
            addPending(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(
                ctx, event_intf::FilterRestored("FILTER_NAME", objectPath));

            removePending(eventName);
        }
    }

    debug("Filter failure event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generatePumpFailureEvent(sdbusplus::object_path objectPath,
                                      bool asserted) -> sdbusplus::async::task<>
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
            addPending(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(
                ctx, event_intf::PumpRestored("PUMP_NAME", objectPath));

            removePending(eventName);
        }
    }

    debug("Pump failure event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generateFanFailureEvent(sdbusplus::object_path objectPath,
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
            addPending(eventName, eventPath);
        }
    }
    else
    {
        if (pendingEvent != pendingEvents.end())
        {
            co_await lg2::resolve(ctx, pendingEvent->second);

            co_await lg2::commit(
                ctx, event_intf::FanRestored("FAN_NAME", objectPath));

            removePending(eventName);
        }
    }

    debug("Fan failure event for {EVENT_NAME} is {STATUS}", "EVENT_NAME",
          eventName, "STATUS", (asserted ? "asserted" : "deasserted"));
}

auto Events::generateLeakDetectedEvent(sdbusplus::object_path objectPath,
                                       EventLevel level, bool asserted)
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

            removePending(eventName);
        }
        co_return;
    }

    namespace error_intf =
        sdbusplus::error::xyz::openbmc_project::state::leak::Detector;
    sdbusplus::object_path eventPath{};

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
