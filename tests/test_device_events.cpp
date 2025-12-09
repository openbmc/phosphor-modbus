#include "common/events.hpp"
#include "device/device_factory.hpp"
#include "modbus_server_tester.hpp"
#include "port/base_port.hpp"
#include "test_base.hpp"

#include <xyz/openbmc_project/Logging/Create/aserver.hpp>
#include <xyz/openbmc_project/Logging/Entry/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Threshold/Critical/client.hpp>
#include <xyz/openbmc_project/Sensor/Value/client.hpp>
#include <xyz/openbmc_project/State/Decorator/Availability/client.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/client.hpp>

#include <cmath>
#include <string>

#include <gtest/gtest.h>

using namespace std::literals;
using namespace testing;
using SensorValueIntf =
    sdbusplus::client::xyz::openbmc_project::sensor::Value<>;
using OperationalStatusIntf = sdbusplus::client::xyz::openbmc_project::state::
    decorator::OperationalStatus<>;
using AvailabilityIntf =
    sdbusplus::client::xyz::openbmc_project::state::decorator::Availability<>;
using ThresholdCriticalIntf =
    sdbusplus::client::xyz::openbmc_project::sensor::threshold::Critical<>;

class TestEventServer;
class TestEventEntry;

using EventServerIntf =
    sdbusplus::aserver::xyz::openbmc_project::logging::Create<TestEventServer>;
using EventEntryIntf =
    sdbusplus::aserver::xyz::openbmc_project::logging::Entry<TestEventEntry>;

namespace ModbusIntf = phosphor::modbus::rtu;
namespace PortIntf = phosphor::modbus::rtu::port;
namespace PortConfigIntf = PortIntf::config;
namespace DeviceIntf = phosphor::modbus::rtu::device;
namespace DeviceConfigIntf = DeviceIntf::config;
namespace EventIntf = phosphor::modbus::events;

class MockPort : public PortIntf::BasePort
{
  public:
    MockPort(sdbusplus::async::context& ctx,
             const PortConfigIntf::Config& config,
             const std::string& devicePath) : BasePort(ctx, config, devicePath)
    {}
};

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
        static int cnt = 100;
        cnt++;

        // Append the count to the object path to make it unique for each event
        std::string objectPath =
            "/xyz/openbmc_project/logging/entry/TestEvent" +
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

    std::string expectedEvent;

  private:
    sdbusplus::async::context& ctx;
    std::vector<std::unique_ptr<TestEventEntry>> eventEntries;
};

class DeviceEventsTest : public BaseTest
{
  public:
    PortConfigIntf::Config portConfig;
    static constexpr const char* clientDevicePath =
        "/tmp/ttyDeviceEventsTestPort0";
    static constexpr const char* serverDevicePath =
        "/tmp/ttyDeviceEventsTestPort1";
    static constexpr auto portName = "TestPort0";
    std::string deviceName;
    std::string fullSensorName;
    std::string objectPath;
    const char* loggingObjectPath = "/xyz/openbmc_project/logging";
    static constexpr auto serviceName = "xyz.openbmc_project.Logging";
    static constexpr auto sensorName = "OutletTemperature";
    TestEventServer eventServer;
    sdbusplus::server::manager_t manager;

    DeviceEventsTest() :
        BaseTest(clientDevicePath, serverDevicePath, serviceName),
        eventServer(ctx, loggingObjectPath), manager(ctx, loggingObjectPath)
    {
        portConfig.name = portName;
        portConfig.portMode = PortConfigIntf::PortMode::rs485;
        portConfig.baudRate = baudRate;
        portConfig.rtsDelay = 1;

        deviceName = std::format("ResorviorPumpUnit_{}_{}",
                                 TestIntf::testDeviceAddress, portName);

        fullSensorName = std::format("{}_{}", deviceName, sensorName);

        objectPath = std::format(
            "{}/{}/{}", SensorValueIntf::namespace_path::value,
            SensorValueIntf::namespace_path::temperature, fullSensorName);
    }

    auto verifyValue(bool currentValue, bool expectedValue,
                     const std::string& failureStr) -> void
    {
        EXPECT_EQ(currentValue, expectedValue) << failureStr;
    }

    auto verifyValue(double currentValue, double expectedValue,
                     const std::string& failureStr) -> void
    {
        EXPECT_EQ(currentValue, expectedValue) << failureStr;
    }

    auto verifyResult(
        SensorValueIntf::properties_t& properties,
        OperationalStatusIntf::properties_t& operationalProperties,
        AvailabilityIntf::properties_t& availabilityProperties,
        ThresholdCriticalIntf::properties_t& thresholdProperties,
        double expectedValue, SensorValueIntf::Unit expectedUnit) -> void
    {
        if (std::isnan(expectedValue))
        {
            EXPECT_TRUE(std::isnan(properties.value))
                << "Sensor value should be Nan";
            verifyValue(operationalProperties.functional, false,
                        "Operational status mismatch");
            verifyValue(availabilityProperties.available, false,
                        "Availability mismatch");
            verifyValue(thresholdProperties.critical_alarm_high, false,
                        "Critical Alarm mismatch");
        }
        else
        {
            verifyValue(properties.value, expectedValue,
                        "Sensor value mismatch");
            verifyValue(operationalProperties.functional, true,
                        "Operational status mismatch");
            verifyValue(availabilityProperties.available, true,
                        "Availability mismatch");
            verifyValue(thresholdProperties.critical_alarm_high, true,
                        "Critical Alarm mismatch");
        }

        EXPECT_EQ(properties.unit, expectedUnit) << "Sensor unit mismatch";
        EXPECT_TRUE(std::isnan(properties.min_value)) << "Min value mismatch";
        EXPECT_TRUE(std::isnan(properties.max_value)) << "Max value mismatch";
    }

    auto testSensorCreation(std::string objectPath,
                            DeviceConfigIntf::StatusType statusType,
                            double expectedValue)
        -> sdbusplus::async::task<void>
    {
        DeviceConfigIntf::StatusBit statusBit = {
            .name = sensorName,
            .type = statusType,
            .bitPosition = 0,
            .value = true};
        DeviceConfigIntf::Config::status_registers_t statusRegisters = {
            {TestIntf::testReadHoldingRegisterEventOffset, {statusBit}}};
        DeviceConfigIntf::Config::sensor_registers_t sensorRegisters = {{
            .name = sensorName,
            .pathSuffix = SensorValueIntf::namespace_path::temperature,
            .unit = SensorValueIntf::Unit::DegreesC,
            .offset = TestIntf::testReadHoldingRegisterTempUnsignedOffset,
            .size = TestIntf::testReadHoldingRegisterTempCount,
            .format = DeviceConfigIntf::SensorFormat::floatingPoint,
        }};
        DeviceConfigIntf::DeviceFactoryConfig deviceFactoryConfig = {
            {
                .address = TestIntf::testDeviceAddress,
                .parity = ModbusIntf::Parity::none,
                .baudRate = baudRate,
                .name = deviceName,
                .portName = portConfig.name,
                .inventoryPath = sdbusplus::message::object_path(
                    "xyz/openbmc_project/Inventory/ResorviorPumpUnit"),
                .sensorRegisters = sensorRegisters,
                .statusRegisters = statusRegisters,
                .firmwareRegisters = {},
            },
            DeviceConfigIntf::DeviceType::reservoirPumpUnit,
            DeviceConfigIntf::DeviceModel::RDF040DSS5193E0,
        };
        EventIntf::Events events{ctx};
        MockPort mockPort(ctx, portConfig, clientDevicePath);
        auto device = DeviceIntf::DeviceFactory::create(
            ctx, deviceFactoryConfig, mockPort, events);
        co_await device->readSensorRegisters();
        auto properties = co_await SensorValueIntf(ctx)
                              .service(serviceName)
                              .path(objectPath)
                              .properties();
        auto operationalProperties =
            co_await OperationalStatusIntf(ctx)
                .service(serviceName)
                .path(objectPath)
                .properties();
        auto availabilityProperties =
            co_await AvailabilityIntf(ctx)
                .service(serviceName)
                .path(objectPath)
                .properties();
        auto thresholdProperties =
            co_await ThresholdCriticalIntf(ctx)
                .service(serviceName)
                .path(objectPath)
                .properties();
        verifyResult(properties, operationalProperties, availabilityProperties,
                     thresholdProperties, expectedValue,
                     sensorRegisters[0].unit);
        co_return;
    }
};

TEST_F(DeviceEventsTest, TestSensorReadingCritical)
{
    eventServer.expectedEvent =
        "xyz.openbmc_project.Sensor.Threshold.ReadingCritical";

    ctx.spawn(testSensorCreation(
        objectPath, DeviceConfigIntf::StatusType::sensorReadingCritical,
        TestIntf::testReadHoldingRegisterTempUnsigned[0]));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(DeviceEventsTest, TestSensorFailure)
{
    eventServer.expectedEvent = "xyz.openbmc_project.Sensor.SensorFailure";

    ctx.spawn(testSensorCreation(objectPath,
                                 DeviceConfigIntf::StatusType::sensorFailure,
                                 std::numeric_limits<double>::quiet_NaN()));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
