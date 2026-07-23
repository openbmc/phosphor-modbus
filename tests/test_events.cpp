#include "common/events.hpp"

#include <nlohmann/json.hpp>
#include <xyz/openbmc_project/Logging/Create/aserver.hpp>
#include <xyz/openbmc_project/Logging/Entry/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/event.hpp>
#include <xyz/openbmc_project/Sensor/event.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/event.hpp>
#include <xyz/openbmc_project/State/Power/event.hpp>
#include <xyz/openbmc_project/State/SMC/event.hpp>

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

class TestEventServer;
class TestEventEntry;

using namespace std::literals;
namespace EventIntf = phosphor::modbus::events;
using EventServerIntf =
    sdbusplus::aserver::xyz::openbmc_project::logging::Create<TestEventServer>;
using EventEntryIntf =
    sdbusplus::aserver::xyz::openbmc_project::logging::Entry<TestEventEntry>;

namespace SensorThresholdErrorIntf =
    sdbusplus::error::xyz::openbmc_project::sensor::Threshold;
namespace SensorThresholdEventIntf =
    sdbusplus::event::xyz::openbmc_project::sensor::Threshold;
namespace SensorErrorIntf = sdbusplus::error::xyz::openbmc_project::Sensor;
namespace SensorEventIntf = sdbusplus::event::xyz::openbmc_project::Sensor;
namespace ControllerErrorIntf =
    sdbusplus::error::xyz::openbmc_project::state::SMC;
namespace ControllerEventIntf =
    sdbusplus::event::xyz::openbmc_project::state::SMC;
namespace PowerErrorIntf = sdbusplus::error::xyz::openbmc_project::state::Power;
namespace PowerEventIntf = sdbusplus::event::xyz::openbmc_project::state::Power;
namespace LeakErrorIntf =
    sdbusplus::error::xyz::openbmc_project::state::leak::Detector;
namespace LeakEventIntf =
    sdbusplus::event::xyz::openbmc_project::state::leak::Detector;

// Test Event Class to mock the EventEntry
class TestEventEntry : public EventEntryIntf
{
  public:
    TestEventEntry(sdbusplus::async::context& ctx, const char* path) :
        EventEntryIntf(ctx, path)
    {}

    auto method_call(get_entry_t)
        -> sdbusplus::async::task<get_entry_t::return_type>
    {
        get_entry_t::return_type fd1 = 0;
        co_return fd1;
    }
};

// Test Event Server Class to mock the EventServer
class TestEventServer : public EventServerIntf
{
  public:
    TestEventServer(sdbusplus::async::context& ctx, const char* path) :
        EventServerIntf(ctx, path), ctx(ctx)
    {}

    auto method_call(create_t, auto message, auto, auto)
        -> sdbusplus::async::task<create_t::return_type>

    {
        static int cnt = 1;
        cnt++;

        // Append the count to the object path to make it unique for each event
        std::string objectPath =
            "/xyz/openbmc_project/logging/entry/TestEvent" +
            std::to_string(cnt);
        EXPECT_EQ(message, expectedEvent) << "Event name mismatch";

        createCount++;
        eventEntries.emplace_back(
            std::make_unique<TestEventEntry>(ctx, objectPath.c_str()));

        co_return sdbusplus::object_path(objectPath);
    }

    auto method_call(create_with_ffdc_files_t, auto, auto, auto, auto)
        -> sdbusplus::async::task<create_with_ffdc_files_t::return_type>

    {
        co_return;
    }

    // Destroy the most recently created entry to simulate the logging
    // service rotating it out ("wrapping").
    void removeLastEntry()
    {
        if (!eventEntries.empty())
        {
            eventEntries.pop_back();
        }
    }

    std::string expectedEvent = "";
    // Cumulative logs created; never decremented.
    int createCount = 0;

  private:
    sdbusplus::async::context& ctx;
    std::vector<std::unique_ptr<TestEventEntry>> eventEntries;
};

class EventsTest : public ::testing::Test
{
  public:
    enum class EventTestType
    {
        sensorWarningEvent,
        sensorCriticalEvent,
        sensorFailureEvent,
        controllerFailureEvent,
        powerFailureEvent,
        leakWarningEvent,
        leakCriticalEvent,
    };

    static constexpr auto sensorObjectPath =
        "/xyz/openbmc_project/sensors/OutletTemperature";
    static constexpr auto serviceName = "xyz.openbmc_project.Logging";
    static constexpr auto assert = true;
    static constexpr auto deassert = false;
    const char* objectPath = "/xyz/openbmc_project/logging";
    sdbusplus::async::context ctx;
    std::filesystem::path stateDir =
        std::filesystem::temp_directory_path() / "phosphor-modbus-test-events";
    EventIntf::Events events;
    TestEventServer eventServer;
    sdbusplus::server::manager_t manager;

    EventsTest() :
        events(ctx, stateDir), eventServer(ctx, objectPath),
        manager(ctx, objectPath)
    {
        std::error_code ec;
        std::filesystem::remove_all(stateDir, ec);
        ctx.request_name(serviceName);
    }

    ~EventsTest() noexcept override {}

    auto testAssertEvent(EventTestType eventType)
        -> sdbusplus::async::task<void>
    {
        switch (eventType)
        {
            case EventTestType::sensorWarningEvent:
                eventServer.expectedEvent =
                    SensorThresholdErrorIntf::ReadingWarning::errName;
                co_await events.generateSensorReadingEvent(
                    sdbusplus::object_path(sensorObjectPath),
                    EventIntf::EventLevel::warning, 60,
                    EventIntf::SensorValueIntf::Unit::DegreesC, assert);
                break;
            case EventTestType::sensorCriticalEvent:
                eventServer.expectedEvent =
                    SensorThresholdErrorIntf::ReadingCritical::errName;
                co_await events.generateSensorReadingEvent(
                    sdbusplus::object_path(sensorObjectPath),
                    EventIntf::EventLevel::critical, 80,
                    EventIntf::SensorValueIntf::Unit::DegreesC, assert);
                break;
            case EventTestType::sensorFailureEvent:
                eventServer.expectedEvent =
                    SensorErrorIntf::SensorFailure::errName;
                co_await events.generateSensorFailureEvent(
                    sdbusplus::object_path(sensorObjectPath), assert);
                break;
            case EventTestType::controllerFailureEvent:
                eventServer.expectedEvent =
                    ControllerErrorIntf::SMCFailed::errName;
                co_await events.generateControllerFailureEvent(
                    sdbusplus::object_path(sensorObjectPath), "", assert);
                break;
            case EventTestType::powerFailureEvent:
                eventServer.expectedEvent =
                    PowerErrorIntf::PowerRailFault::errName;
                co_await events.generatePowerFaultEvent(
                    sdbusplus::object_path(sensorObjectPath), "", assert);
                break;
            case EventTestType::leakWarningEvent:
                eventServer.expectedEvent =
                    LeakErrorIntf::LeakDetectedWarning::errName;
                co_await events.generateLeakDetectedEvent(
                    sdbusplus::object_path(sensorObjectPath),
                    EventIntf::EventLevel::warning, assert);
                break;
            case EventTestType::leakCriticalEvent:
                eventServer.expectedEvent =
                    LeakErrorIntf::LeakDetectedCritical::errName;
                co_await events.generateLeakDetectedEvent(
                    sdbusplus::object_path(sensorObjectPath),
                    EventIntf::EventLevel::critical, assert);
                break;
        }
    }

    auto testDeassertEvent(EventTestType eventType)
        -> sdbusplus::async::task<void>
    {
        switch (eventType)
        {
            case EventTestType::sensorWarningEvent:
                eventServer.expectedEvent =
                    SensorThresholdEventIntf::SensorReadingNormalRange::errName;
                co_await events.generateSensorReadingEvent(
                    sdbusplus::object_path(sensorObjectPath),
                    EventIntf::EventLevel::warning, 40,
                    EventIntf::SensorValueIntf::Unit::DegreesC, deassert);
                break;
            case EventTestType::sensorCriticalEvent:
                eventServer.expectedEvent =
                    SensorThresholdEventIntf::SensorReadingNormalRange::errName;
                co_await events.generateSensorReadingEvent(
                    sdbusplus::object_path(sensorObjectPath),
                    EventIntf::EventLevel::critical, 40,
                    EventIntf::SensorValueIntf::Unit::DegreesC, deassert);
                break;
            case EventTestType::sensorFailureEvent:
                eventServer.expectedEvent =
                    SensorEventIntf::SensorRestored::errName;
                co_await events.generateSensorFailureEvent(
                    sdbusplus::object_path(sensorObjectPath), deassert);
                break;
            case EventTestType::controllerFailureEvent:
                eventServer.expectedEvent =
                    ControllerEventIntf::SMCRestored::errName;
                co_await events.generateControllerFailureEvent(
                    sdbusplus::object_path(sensorObjectPath), "", deassert);
                break;
            case EventTestType::powerFailureEvent:
                eventServer.expectedEvent =
                    PowerEventIntf::PowerRailFaultRecovered::errName;
                co_await events.generatePowerFaultEvent(
                    sdbusplus::object_path(sensorObjectPath), "", deassert);
                break;
            case EventTestType::leakWarningEvent:
                eventServer.expectedEvent =
                    LeakEventIntf::LeakDetectedNormal::errName;
                co_await events.generateLeakDetectedEvent(
                    sdbusplus::object_path(sensorObjectPath),
                    EventIntf::EventLevel::warning, deassert);
                break;
            case EventTestType::leakCriticalEvent:
                eventServer.expectedEvent =
                    LeakEventIntf::LeakDetectedNormal::errName;
                co_await events.generateLeakDetectedEvent(
                    sdbusplus::object_path(sensorObjectPath),
                    EventIntf::EventLevel::critical, deassert);
                break;
        }
    }

    auto testEvents(EventTestType eventType) -> sdbusplus::async::task<void>
    {
        co_await testAssertEvent(eventType);

        co_await sdbusplus::async::sleep_for(ctx, 1s);

        co_await testDeassertEvent(eventType);

        ctx.request_stop();
    }

    // Asserting an event persists its log path; a fresh Events instance
    // restores it from disk, so the matching deassert resolves the persisted
    // entry across a simulated restart instead of orphaning the log.
    auto testPersistRestore() -> sdbusplus::async::task<void>
    {
        co_await testAssertEvent(EventTestType::sensorWarningEvent);

        auto stateFilePath = stateDir / "pending-events.json";
        EXPECT_TRUE(std::filesystem::exists(stateFilePath));

        auto eventName = std::string(sensorObjectPath) + ".threshold.Warning";
        {
            std::ifstream in(stateFilePath);
            auto j = nlohmann::json::parse(in);
            EXPECT_EQ(j.value("version", ""), "1.0.0");
            auto pending = j.value("pendingEvents", nlohmann::json::object());
            EXPECT_TRUE(pending.contains(eventName));
            if (pending.contains(eventName))
            {
                EXPECT_FALSE(pending[eventName].get<std::string>().empty());
            }
        }

        // Simulate a restart: a new Events restores the pending map from disk.
        EventIntf::Events restored{ctx, stateDir};
        restored.restore();

        eventServer.expectedEvent =
            SensorThresholdEventIntf::SensorReadingNormalRange::errName;
        co_await restored.generateSensorReadingEvent(
            sdbusplus::object_path(sensorObjectPath),
            EventIntf::EventLevel::warning, 40,
            EventIntf::SensorValueIntf::Unit::DegreesC, deassert);

        // Resolved: the entry is dropped from the persisted file.
        {
            std::ifstream in(stateFilePath);
            auto j = nlohmann::json::parse(in);
            auto pending = j.value("pendingEvents", nlohmann::json::object());
            EXPECT_FALSE(pending.contains(eventName));
        }

        ctx.request_stop();
    }

    // A tracked log that is rotated out ("wrapped") must be regenerated on the
    // next assert, while a still-present log is not duplicated.
    auto testWrapRecovery() -> sdbusplus::async::task<void>
    {
        eventServer.expectedEvent =
            SensorThresholdErrorIntf::ReadingWarning::errName;

        // First assert creates the log.
        co_await events.generateSensorReadingEvent(
            sdbusplus::object_path(sensorObjectPath),
            EventIntf::EventLevel::warning, 60,
            EventIntf::SensorValueIntf::Unit::DegreesC, assert);
        EXPECT_EQ(eventServer.createCount, 1);

        // Re-asserting while the log still exists must not duplicate it.
        co_await events.generateSensorReadingEvent(
            sdbusplus::object_path(sensorObjectPath),
            EventIntf::EventLevel::warning, 60,
            EventIntf::SensorValueIntf::Unit::DegreesC, assert);
        EXPECT_EQ(eventServer.createCount, 1);

        // Simulate the logging service wrapping the entry out.
        eventServer.removeLastEntry();

        // Re-asserting now must regenerate the lost log.
        co_await events.generateSensorReadingEvent(
            sdbusplus::object_path(sensorObjectPath),
            EventIntf::EventLevel::warning, 60,
            EventIntf::SensorValueIntf::Unit::DegreesC, assert);
        EXPECT_EQ(eventServer.createCount, 2);

        ctx.request_stop();
    }
};

TEST_F(EventsTest, TestEventsSensorWarning)
{
    ctx.spawn(testEvents(EventTestType::sensorWarningEvent));
    ctx.run();
}

TEST_F(EventsTest, TestEventsSensorCritical)
{
    ctx.spawn(testEvents(EventTestType::sensorCriticalEvent));
    ctx.run();
}

TEST_F(EventsTest, TestEventsSensorFailure)
{
    ctx.spawn(testEvents(EventTestType::sensorFailureEvent));
    ctx.run();
}

TEST_F(EventsTest, TestEventsControllerFailure)
{
    ctx.spawn(testEvents(EventTestType::controllerFailureEvent));
    ctx.run();
}

TEST_F(EventsTest, TestEventsPowerFailure)
{
    ctx.spawn(testEvents(EventTestType::powerFailureEvent));
    ctx.run();
}

TEST_F(EventsTest, TestEventsLeakWarning)
{
    ctx.spawn(testEvents(EventTestType::leakWarningEvent));
    ctx.run();
}

TEST_F(EventsTest, TestEventsLeakCritical)
{
    ctx.spawn(testEvents(EventTestType::leakCriticalEvent));
    ctx.run();
}

TEST_F(EventsTest, TestPendingEventsPersistAndRestore)
{
    ctx.spawn(testPersistRestore());
    ctx.run();
}

TEST_F(EventsTest, TestPendingEventWrapRecovery)
{
    ctx.spawn(testWrapRecovery());
    ctx.run();
}
