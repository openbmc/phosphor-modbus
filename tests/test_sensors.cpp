#include "common/events.hpp"
#include "device/device_factory.hpp"
#include "modbus_rtu_config.hpp"
#include "port/base_port.hpp"
#include "test_base.hpp"

#include <xyz/openbmc_project/Association/Definitions/client.hpp>
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
using AssociationIntf =
    sdbusplus::client::xyz::openbmc_project::association::Definitions<>;

namespace ModbusIntf = phosphor::modbus::rtu;
namespace ProfileIntf = phosphor::modbus::rtu::profile;
namespace PortIntf = phosphor::modbus::rtu::port;
namespace PortConfigIntf = PortIntf::config;
namespace DeviceIntf = phosphor::modbus::rtu::device;
namespace DeviceConfigIntf = DeviceIntf::config;
namespace EventIntf = phosphor::modbus::events;
using SensorTypeIntf = ProfileIntf::SensorType;

class MockPort : public PortIntf::BasePort
{
  public:
    MockPort(sdbusplus::async::context& ctx,
             const PortConfigIntf::Config& config,
             const std::string& devicePath) : BasePort(ctx, config, devicePath)
    {}
};

struct DeviceTestConfig
{
    std::string devicePrefix;
    std::string inventoryPath;
    ProfileIntf::DeviceType deviceType;
    ProfileIntf::DeviceModel deviceModel;
};

class SensorsTest : public BaseTest
{
  public:
    static constexpr auto clientPathPrefix = "/tmp/ttySensorsTestPort0";
    static constexpr auto serverPathPrefix = "/tmp/ttySensorsTestPort1";
    static constexpr auto portName = "TestPort0";
    static constexpr auto serviceName =
        "xyz.openbmc_project.TestModbusRTUSensors";
    static constexpr auto sensorName = "OutletTemperature";

    PortConfigIntf::Config portConfig;
    DeviceTestConfig deviceTestConfig;
    std::string deviceName;
    std::string fullSensorName;
    std::string objectPath;

    void setupDevice(const DeviceTestConfig& config)
    {
        deviceTestConfig = config;

        deviceName = std::format("{}_{}_{}", deviceTestConfig.devicePrefix,
                                 TestIntf::testDeviceAddress, portName);

        fullSensorName = std::format("{}_{}", deviceName, sensorName);

        objectPath = std::format(
            "{}/{}/{}", SensorValueIntf::namespace_path::value,
            SensorValueIntf::namespace_path::temperature, fullSensorName);
    }

    auto getSensorObjectPath(const std::string& name, SensorTypeIntf sensorType)
        -> std::string
    {
        return std::format(
            "{}/{}/{}_{}", SensorValueIntf::namespace_path::value,
            DeviceIntf::getPathSuffix(sensorType), deviceName, name);
    }

    SensorsTest() : BaseTest(clientPathPrefix, serverPathPrefix, serviceName)
    {
        portConfig.name = portName;
        portConfig.portMode = PortConfigIntf::PortMode::rs485;
        portConfig.baudRate = baudRate;
        portConfig.rtsDelay = 1;
    }

    auto checkInventoryAssociations() -> sdbusplus::async::task<void>
    {
        constexpr auto numOfInventoryAssociations = 3;
        auto associationProperties =
            co_await AssociationIntf(ctx)
                .service(serviceName)
                .path(objectPath)
                .properties();
        EXPECT_EQ(associationProperties.associations.size(),
                  numOfInventoryAssociations);
    }

    auto createDevice(std::vector<ProfileIntf::SensorRegister> sensorRegisters,
                      EventIntf::Events& events)
        -> std::pair<std::unique_ptr<MockPort>,
                     std::unique_ptr<DeviceIntf::BaseDevice>>
    {
        testProfile.sensorRegisters = std::move(sensorRegisters);

        DeviceConfigIntf::DeviceFactoryConfig deviceFactoryConfig = {
            {
                .name = deviceName,
                .type = "TestDevice",
                .address = TestIntf::testDeviceAddress,
                .serialPort = portConfig.name,
                .inventoryPath =
                    sdbusplus::object_path(deviceTestConfig.inventoryPath),
                .profile = testProfile,
            },
            deviceTestConfig.deviceType,
            deviceTestConfig.deviceModel,
        };
        auto mockPort =
            std::make_unique<MockPort>(ctx, portConfig, clientDevicePath);
        auto device = DeviceIntf::DeviceFactory::create(
            ctx, deviceFactoryConfig, *mockPort, events);
        return {std::move(mockPort), std::move(device)};
    }

    auto testSensorCreation(std::string objectPath,
                            ProfileIntf::SensorRegister sensorRegister,
                            double expectedValue)
        -> sdbusplus::async::task<void>
    {
        EventIntf::Events events{ctx};
        auto devPair = createDevice({sensorRegister}, events);
        auto& device = devPair.second;

        co_await device->readSensorRegisters();

        auto properties = co_await SensorValueIntf(ctx)
                              .service(serviceName)
                              .path(objectPath)
                              .properties();
        EXPECT_EQ(properties.value, expectedValue) << "Sensor value mismatch";
        EXPECT_EQ(properties.unit, DeviceIntf::getUnit(sensorRegister.type))
            << "Sensor unit mismatch";
        EXPECT_TRUE(std::isnan(properties.min_value)) << "Min value mismatch";
        EXPECT_TRUE(std::isnan(properties.max_value)) << "Max value mismatch";

        auto operationalProperties =
            co_await OperationalStatusIntf(ctx)
                .service(serviceName)
                .path(objectPath)
                .properties();
        EXPECT_EQ(operationalProperties.functional, true)
            << "Operational status mismatch";

        auto availabilityProperties =
            co_await AvailabilityIntf(ctx)
                .service(serviceName)
                .path(objectPath)
                .properties();
        EXPECT_EQ(availabilityProperties.available, true)
            << "Availability mismatch";

        co_await checkInventoryAssociations();

        co_return;
    }

    ProfileIntf::DeviceProfile testProfile = {
        .parity = ModbusIntf::Parity::none,
        .baudRate = baudRate,
        .probeRegister = {},
        .inventoryRegisters = {},
        .sensorRegisters = {},
        .statusRegisters = {},
        .firmwareRegisters = {},
    };
};

TEST_F(SensorsTest, TestRpuSensorValueUnsigned)
{
    setupDevice({
        "ResorviorPumpUnit",
        "xyz/openbmc_project/Inventory/ResorviorPumpUnit",
        ProfileIntf::DeviceType::reservoirPumpUnit,
        ProfileIntf::DeviceModel::DeltaRDF040DSS5193E0,
    });

    const ProfileIntf::SensorRegister sensorRegister = {
        .name = sensorName,
        .type = SensorTypeIntf::temperature,
        .offset = TestIntf::testReadHoldingRegisterTempUnsignedOffset,
        .size = TestIntf::testReadHoldingRegisterTempCount,
        .format = ProfileIntf::SensorFormat::floatingPoint,
    };

    ctx.spawn(
        testSensorCreation(objectPath, sensorRegister,
                           TestIntf::testReadHoldingRegisterTempUnsigned[0]));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(SensorsTest, TestRpuSensorValueSigned)
{
    setupDevice({
        "ResorviorPumpUnit",
        "xyz/openbmc_project/Inventory/ResorviorPumpUnit",
        ProfileIntf::DeviceType::reservoirPumpUnit,
        ProfileIntf::DeviceModel::DeltaRDF040DSS5193E0,
    });

    const ProfileIntf::SensorRegister sensorRegister = {
        .name = sensorName,
        .type = SensorTypeIntf::temperature,
        .offset = TestIntf::testReadHoldingRegisterTempSignedOffset,
        .size = TestIntf::testReadHoldingRegisterTempCount,
        .isSigned = true,
        .format = ProfileIntf::SensorFormat::floatingPoint,
    };

    // Convert expected hex value to a signed 16-bit integer for comparison
    const int16_t expectedSigned =
        static_cast<int16_t>(TestIntf::testReadHoldingRegisterTempSigned[0]);

    ctx.spawn(testSensorCreation(objectPath, sensorRegister, expectedSigned));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

static auto applyValueSettings(double value, double shift, double scale,
                               uint8_t precision)
{
    return (shift + (scale * (value / (1ULL << precision))));
}

TEST_F(SensorsTest, TestRpuSensorValueWithSettings)
{
    setupDevice({
        "ResorviorPumpUnit",
        "xyz/openbmc_project/Inventory/ResorviorPumpUnit",
        ProfileIntf::DeviceType::reservoirPumpUnit,
        ProfileIntf::DeviceModel::DeltaRDF040DSS5193E0,
    });

    const ProfileIntf::SensorRegister sensorRegister = {
        .name = sensorName,
        .type = SensorTypeIntf::temperature,
        .offset = TestIntf::testReadHoldingRegisterTempUnsignedOffset,
        .size = TestIntf::testReadHoldingRegisterTempCount,
        .precision = 2,
        .scale = 0.1,
        .shift = 50,
        .format = ProfileIntf::SensorFormat::floatingPoint,
    };

    ctx.spawn(testSensorCreation(
        objectPath, sensorRegister,
        applyValueSettings(TestIntf::testReadHoldingRegisterTempUnsigned[0],
                           sensorRegister.shift, sensorRegister.scale,
                           sensorRegister.precision)));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(SensorsTest, TestPmmSensorValueUnsigned)
{
    setupDevice({
        "PowerMonitorModule",
        "xyz/openbmc_project/Inventory/PowerMonitorModule",
        ProfileIntf::DeviceType::powerMonitorModule,
        ProfileIntf::DeviceModel::PanasonicBJBPM102A0001,
    });

    const ProfileIntf::SensorRegister sensorRegister = {
        .name = sensorName,
        .type = SensorTypeIntf::temperature,
        .offset = TestIntf::testReadHoldingRegisterTempUnsignedOffset,
        .size = TestIntf::testReadHoldingRegisterTempCount,
        .format = ProfileIntf::SensorFormat::floatingPoint,
    };

    ctx.spawn(
        testSensorCreation(objectPath, sensorRegister,
                           TestIntf::testReadHoldingRegisterTempUnsigned[0]));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

// Two contiguous registers (0x0120, 0x0121) should be merged into a single
// span and read in one Modbus transaction.
TEST_F(SensorsTest, TestContiguousRegistersSpanMerge)
{
    setupDevice({
        "ResorviorPumpUnit",
        "xyz/openbmc_project/Inventory/ResorviorPumpUnit",
        ProfileIntf::DeviceType::reservoirPumpUnit,
        ProfileIntf::DeviceModel::DeltaRDF040DSS5193E0,
    });

    const std::string sensor1Name = "Sensor1";
    const std::string sensor2Name = "Sensor2";

    std::vector<ProfileIntf::SensorRegister> sensorRegisters = {
        {.name = sensor1Name,
         .type = SensorTypeIntf::temperature,
         .offset = TestIntf::testReadHoldingRegisterSpanSensor1Offset,
         .size = 1,
         .format = ProfileIntf::SensorFormat::floatingPoint,
         .pollInterval = ModbusIntf::defaultSensorPollInterval},
        {.name = sensor2Name,
         .type = SensorTypeIntf::temperature,
         .offset = TestIntf::testReadHoldingRegisterSpanSensor2Offset,
         .size = 1,
         .format = ProfileIntf::SensorFormat::floatingPoint,
         .pollInterval = ModbusIntf::defaultSensorPollInterval}};

    auto testSpan = [&]() -> sdbusplus::async::task<void> {
        EventIntf::Events events{ctx};
        auto devPair = createDevice(sensorRegisters, events);
        auto& device = devPair.second;
        auto countBefore = serverTester->totalRequestCount.load();
        co_await device->readSensorRegisters();

        // Two contiguous registers should merge into 1 Modbus read.
        EXPECT_EQ(serverTester->totalRequestCount.load() - countBefore, 1)
            << "Expected single merged read for contiguous registers";

        auto path1 =
            getSensorObjectPath(sensor1Name, SensorTypeIntf::temperature);
        auto props1 = co_await SensorValueIntf(ctx)
                          .service(serviceName)
                          .path(path1)
                          .properties();
        EXPECT_EQ(props1.value, TestIntf::testReadHoldingRegisterSpanMerged[0])
            << "Sensor1 value mismatch";

        auto path2 =
            getSensorObjectPath(sensor2Name, SensorTypeIntf::temperature);
        auto props2 = co_await SensorValueIntf(ctx)
                          .service(serviceName)
                          .path(path2)
                          .properties();
        EXPECT_EQ(props2.value, TestIntf::testReadHoldingRegisterSpanMerged[1])
            << "Sensor2 value mismatch";

        co_return;
    };

    ctx.spawn(testSpan());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

// Two registers far apart (0x0113 and 0x0200) should NOT merge — each gets
// its own span and a separate Modbus read transaction.
TEST_F(SensorsTest, TestDistantRegistersSeparateSpans)
{
    setupDevice({
        "ResorviorPumpUnit",
        "xyz/openbmc_project/Inventory/ResorviorPumpUnit",
        ProfileIntf::DeviceType::reservoirPumpUnit,
        ProfileIntf::DeviceModel::DeltaRDF040DSS5193E0,
    });

    const std::string nearName = "NearSensor";
    const std::string farName = "FarSensor";

    std::vector<ProfileIntf::SensorRegister> sensorRegisters = {
        {.name = nearName,
         .type = SensorTypeIntf::temperature,
         .offset = TestIntf::testReadHoldingRegisterTempUnsignedOffset,
         .size = 1,
         .format = ProfileIntf::SensorFormat::floatingPoint,
         .pollInterval = ModbusIntf::defaultSensorPollInterval},
        {.name = farName,
         .type = SensorTypeIntf::temperature,
         .offset = TestIntf::testReadHoldingRegisterDistantOffset,
         .size = 1,
         .format = ProfileIntf::SensorFormat::floatingPoint,
         .pollInterval = ModbusIntf::defaultSensorPollInterval}};

    auto testSpan = [&]() -> sdbusplus::async::task<void> {
        EventIntf::Events events{ctx};
        auto devPair = createDevice(sensorRegisters, events);
        auto& device = devPair.second;
        auto countBefore = serverTester->totalRequestCount.load();
        co_await device->readSensorRegisters();

        // Two distant registers should produce 2 separate Modbus reads.
        EXPECT_EQ(serverTester->totalRequestCount.load() - countBefore, 2)
            << "Expected two separate reads for distant registers";

        auto nearPath =
            getSensorObjectPath(nearName, SensorTypeIntf::temperature);
        auto nearProps = co_await SensorValueIntf(ctx)
                             .service(serviceName)
                             .path(nearPath)
                             .properties();
        EXPECT_EQ(nearProps.value,
                  TestIntf::testReadHoldingRegisterTempUnsigned[0])
            << "Near sensor value mismatch";

        auto farPath =
            getSensorObjectPath(farName, SensorTypeIntf::temperature);
        auto farProps = co_await SensorValueIntf(ctx)
                            .service(serviceName)
                            .path(farPath)
                            .properties();
        EXPECT_EQ(farProps.value, TestIntf::testReadHoldingRegisterDistant[0])
            << "Far sensor value mismatch";

        co_return;
    };

    ctx.spawn(testSpan());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

// Sensors with different poll intervals should be placed in separate buckets.
// The faster bucket (1s) should poll more often than the slower one (10s)
// within the test window.
TEST_F(SensorsTest, TestDifferentPollIntervalBuckets)
{
    setupDevice({
        "ResorviorPumpUnit",
        "xyz/openbmc_project/Inventory/ResorviorPumpUnit",
        ProfileIntf::DeviceType::reservoirPumpUnit,
        ProfileIntf::DeviceModel::DeltaRDF040DSS5193E0,
    });

    const std::string fastName = "FastSensor";
    const std::string slowName = "SlowSensor";

    std::vector<ProfileIntf::SensorRegister> sensorRegisters = {
        {.name = fastName,
         .type = SensorTypeIntf::temperature,
         .offset = TestIntf::testReadHoldingRegisterTempUnsignedOffset,
         .size = 1,
         .format = ProfileIntf::SensorFormat::floatingPoint,
         .pollInterval = std::chrono::seconds(1)},
        {.name = slowName,
         .type = SensorTypeIntf::temperature,
         .offset = TestIntf::testReadHoldingRegisterDistantOffset,
         .size = 1,
         .format = ProfileIntf::SensorFormat::floatingPoint,
         .pollInterval = std::chrono::seconds(10)}};

    EventIntf::Events events{ctx};
    auto devPair = createDevice(sensorRegisters, events);
    auto& device = devPair.second;
    auto countBefore = serverTester->totalRequestCount.load();

    // Spawn the poll loop and let it run for ~2.5s.
    // Fast bucket (1s): polls at t=0, t=1, t=2 -> 3 requests
    // Slow bucket (10s): polls at t=0 only     -> 1 request
    // Total expected: 4
    ctx.spawn(device->readSensorRegisters());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 2500ms) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();

    auto totalRequests = serverTester->totalRequestCount.load() - countBefore;

    // The fast bucket (1s) should poll multiple times while the slow bucket
    // (10s) polls only once. If both were in a single bucket they would
    // always poll together, producing exactly 2 requests per iteration.
    // With separate buckets we expect more than 2 total (fast polls extra)
    // but fewer than if both polled every second (which would be ~6).
    EXPECT_GT(totalRequests, 2) << "Fast bucket should poll more than once";
    EXPECT_LE(totalRequests, 6)
        << "Slow bucket should not poll more than once in 2.5s";
}

// Verify that calling stop() causes the sensor polling coroutine to exit,
// sets isFinished() to true, and no further sensor polls occur.
TEST_F(SensorsTest, TestStopDeviceExitsAndStopsPolling)
{
    setupDevice({
        "ResorviorPumpUnit",
        "xyz/openbmc_project/Inventory/ResorviorPumpUnit",
        ProfileIntf::DeviceType::reservoirPumpUnit,
        ProfileIntf::DeviceModel::DeltaRDF040DSS5193E0,
    });

    const ProfileIntf::SensorRegister sensorRegister = {
        .name = sensorName,
        .type = SensorTypeIntf::temperature,
        .offset = TestIntf::testReadHoldingRegisterTempUnsignedOffset,
        .size = TestIntf::testReadHoldingRegisterTempCount,
        .format = ProfileIntf::SensorFormat::floatingPoint,
        .pollInterval = std::chrono::seconds(1),
    };

    EventIntf::Events events{ctx};
    auto devPair = createDevice({sensorRegister}, events);
    auto& device = devPair.second;

    EXPECT_FALSE(device->isFinished()) << "Device should not be finished yet";

    ctx.spawn(device->readSensorRegisters());

    // Let it poll once, then stop it
    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1500ms) |
              sdbusplus::async::execution::then([&]() { device->stop(); }));

    // Wait for coroutine to exit, then verify no further polls
    ctx.spawn(sdbusplus::async::sleep_for(ctx, 3s) |
              sdbusplus::async::execution::then([&]() {
                  EXPECT_TRUE(device->isFinished())
                      << "Device should be finished after stop";
                  auto countAfterStop = serverTester->totalRequestCount.load();

                  // Schedule another check — count should not increase
                  ctx.spawn(
                      sdbusplus::async::sleep_for(ctx, 2s) |
                      sdbusplus::async::execution::then([&, countAfterStop]() {
                          EXPECT_EQ(serverTester->totalRequestCount.load(),
                                    countAfterStop)
                              << "No further polls after stop";
                          ctx.request_stop();
                      }));
              }));

    ctx.run();
}

// Two contiguous sensors are merged into one span. The second sensor's offset
// falls on an illegal data address, causing the server to respond with
// Illegal Data Address (0x02). The entire span should fail — both sensors
// should be NaN and non-functional.
TEST_F(SensorsTest, TestIllegalDataAddressFailsEntireSpan)
{
    setupDevice({
        "ResorviorPumpUnit",
        "xyz/openbmc_project/Inventory/ResorviorPumpUnit",
        ProfileIntf::DeviceType::reservoirPumpUnit,
        ProfileIntf::DeviceModel::DeltaRDF040DSS5193E0,
    });

    const std::string validSensorName = "ValidSensor";
    const std::string badSensorName = "BadAddressSensor";
    constexpr uint16_t validOffset =
        TestIntf::testIllegalDataAddressRegister - 1;

    std::vector<ProfileIntf::SensorRegister> sensorRegisters = {
        {.name = validSensorName,
         .type = SensorTypeIntf::temperature,
         .offset = validOffset,
         .size = 1,
         .format = ProfileIntf::SensorFormat::floatingPoint,
         .pollInterval = ModbusIntf::defaultSensorPollInterval},
        {.name = badSensorName,
         .type = SensorTypeIntf::temperature,
         .offset = TestIntf::testIllegalDataAddressRegister,
         .size = 1,
         .format = ProfileIntf::SensorFormat::floatingPoint,
         .pollInterval = ModbusIntf::defaultSensorPollInterval}};

    auto testIllegalAddr = [&]() -> sdbusplus::async::task<void> {
        EventIntf::Events events{ctx};
        auto devPair = createDevice(sensorRegisters, events);
        auto& device = devPair.second;
        co_await device->readSensorRegisters();

        // Both sensors in the span should be NaN and non-functional
        for (const auto& name : {validSensorName, badSensorName})
        {
            auto sensorPath =
                getSensorObjectPath(name, SensorTypeIntf::temperature);
            auto sensorProps = co_await SensorValueIntf(ctx)
                                   .service(serviceName)
                                   .path(sensorPath)
                                   .properties();
            EXPECT_TRUE(std::isnan(sensorProps.value))
                << name << " should be NaN after illegal data address";

            auto opProps = co_await OperationalStatusIntf(ctx)
                               .service(serviceName)
                               .path(sensorPath)
                               .properties();
            EXPECT_FALSE(opProps.functional)
                << name
                << " should be non-functional after illegal data address";
        }

        co_return;
    };

    ctx.spawn(testIllegalAddr());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
