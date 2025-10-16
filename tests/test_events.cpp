#include "common/events.hpp"

#include <xyz/openbmc_project/Logging/Create/aserver.hpp>
#include <xyz/openbmc_project/Logging/Entry/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/event.hpp>
#include <xyz/openbmc_project/Sensor/event.hpp>
#include <xyz/openbmc_project/State/Leak/Detector/event.hpp>
#include <xyz/openbmc_project/State/Power/event.hpp>
#include <xyz/openbmc_project/State/SMC/event.hpp>

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
            "/xyz/openbmc_project/logging/entry/TestEvent1" +
            std::to_string(cnt);
        EXPECT_EQ(message, expectedEvent) << "Event name mismatch";

        eventEntries.emplace_back(
            std::make_unique<TestEventEntry>(ctx, objectPath.c_str()));

        co_return sdbusplus::message::object_path(objectPath);
    }

    auto method_call(create_with_ffdc_files_t, auto, auto, auto, auto)
        -> sdbusplus::async::task<create_with_ffdc_files_t::return_type>

    {
        co_return;
    }

    std::string expectedEvent = "";

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
    EventIntf::Events events;
    TestEventServer eventServer;
    sdbusplus::server::manager_t manager;

    EventsTest() :
        events(ctx), eventServer(ctx, objectPath), manager(ctx, objectPath)
    {
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
                    sdbusplus::message::object_path(sensorObjectPath),
                    EventIntf::EventLevel::warning, 60,
                    EventIntf::SensorValueIntf::Unit::DegreesC, assert);
                break;
            case EventTestType::sensorCriticalEvent:
                eventServer.expectedEvent =
                    SensorThresholdErrorIntf::ReadingCritical::errName;
                co_await events.generateSensorReadingEvent(
                    sdbusplus::message::object_path(sensorObjectPath),
                    EventIntf::EventLevel::critical, 80,
                    EventIntf::SensorValueIntf::Unit::DegreesC, assert);
                break;
            case EventTestType::sensorFailureEvent:
                eventServer.expectedEvent =
                    SensorErrorIntf::SensorFailure::errName;
                co_await events.generateSensorFailureEvent(
                    sdbusplus::message::object_path(sensorObjectPath), assert);
                break;
            case EventTestType::controllerFailureEvent:
                eventServer.expectedEvent =
                    ControllerErrorIntf::SMCFailed::errName;
                co_await events.generateControllerFailureEvent(
                    sdbusplus::message::object_path(sensorObjectPath), "",
                    assert);
                break;
            case EventTestType::powerFailureEvent:
                eventServer.expectedEvent =
                    PowerErrorIntf::PowerRailFault::errName;
                co_await events.generatePowerFaultEvent(
                    sdbusplus::message::object_path(sensorObjectPath), "",
                    assert);
                break;
            case EventTestType::leakWarningEvent:
                eventServer.expectedEvent =
                    LeakErrorIntf::LeakDetectedWarning::errName;
                co_await events.generateLeakDetectedEvent(
                    sdbusplus::message::object_path(sensorObjectPath),
                    EventIntf::EventLevel::warning, assert);
                break;
            case EventTestType::leakCriticalEvent:
                eventServer.expectedEvent =
                    LeakErrorIntf::LeakDetectedCritical::errName;
                co_await events.generateLeakDetectedEvent(
                    sdbusplus::message::object_path(sensorObjectPath),
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
                    sdbusplus::message::object_path(sensorObjectPath),
                    EventIntf::EventLevel::warning, 40,
                    EventIntf::SensorValueIntf::Unit::DegreesC, deassert);
                break;
            case EventTestType::sensorCriticalEvent:
                eventServer.expectedEvent =
                    SensorThresholdEventIntf::SensorReadingNormalRange::errName;
                co_await events.generateSensorReadingEvent(
                    sdbusplus::message::object_path(sensorObjectPath),
                    EventIntf::EventLevel::critical, 40,
                    EventIntf::SensorValueIntf::Unit::DegreesC, deassert);
                break;
            case EventTestType::sensorFailureEvent:
                eventServer.expectedEvent =
                    SensorEventIntf::SensorRestored::errName;
                co_await events.generateSensorFailureEvent(
                    sdbusplus::message::object_path(sensorObjectPath),
                    deassert);
                break;
            case EventTestType::controllerFailureEvent:
                eventServer.expectedEvent =
                    ControllerEventIntf::SMCRestored::errName;
                co_await events.generateControllerFailureEvent(
                    sdbusplus::message::object_path(sensorObjectPath), "",
                    deassert);
                break;
            case EventTestType::powerFailureEvent:
                eventServer.expectedEvent =
                    PowerEventIntf::PowerRailFaultRecovered::errName;
                co_await events.generatePowerFaultEvent(
                    sdbusplus::message::object_path(sensorObjectPath), "",
                    deassert);
                break;
            case EventTestType::leakWarningEvent:
                eventServer.expectedEvent =
                    LeakEventIntf::LeakDetectedNormal::errName;
                co_await events.generateLeakDetectedEvent(
                    sdbusplus::message::object_path(sensorObjectPath),
                    EventIntf::EventLevel::warning, deassert);
                break;
            case EventTestType::leakCriticalEvent:
                eventServer.expectedEvent =
                    LeakEventIntf::LeakDetectedNormal::errName;
                co_await events.generateLeakDetectedEvent(
                    sdbusplus::message::object_path(sensorObjectPath),
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
