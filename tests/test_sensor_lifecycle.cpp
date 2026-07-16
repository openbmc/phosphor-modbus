#include "test_sensor_base.hpp"

// Device lifecycle: port-busy handling, per-register poll-rate overrides, and
// clean stop of the polling coroutine.
class SensorLifecycleTest : public SensorTestBase
{
  public:
    SensorLifecycleTest() :
        SensorTestBase("/tmp/ttySensorLifecyclePort0",
                       "/tmp/ttySensorLifecyclePort1",
                       "xyz.openbmc_project.TestModbusRTUSensorLifecycle")
    {}

    // Sensor is unavailable with a blanked (NaN) reading, but not faulted.
    auto verifyUnavailableWhileBusy(const std::string& path)
        -> sdbusplus::async::task<void>
    {
        EXPECT_TRUE(co_await waitForAvailability(path, false));
        auto op = co_await OperationalStatusIntf(ctx)
                      .service(serviceName)
                      .path(path)
                      .properties();
        EXPECT_TRUE(op.functional) << "busy must not fault the sensor";
        auto val = co_await SensorValueIntf(ctx)
                       .service(serviceName)
                       .path(path)
                       .properties();
        EXPECT_TRUE(std::isnan(val.value))
            << "reading should be NaN while busy";
        co_return;
    }

    auto testPortBusy(std::string objectPath,
                      ProfileIntf::SensorRegister sensorRegister,
                      double expectedValue) -> sdbusplus::async::task<void>
    {
        EventIntf::Events events{ctx};
        auto devPair = createDevice({sensorRegister}, events);
        auto& mockPort = devPair.first;
        auto& device = devPair.second;

        ctx.spawn(device->pollRegisters());

        // First poll succeeds: sensor reads its value and is available.
        EXPECT_TRUE(co_await waitForValue(objectPath, expectedValue));

        // Reserve the port; subsequent polls return busy.
        auto lock = mockPort->acquireExclusive();
        EXPECT_TRUE(lock.has_value());

        co_await verifyUnavailableWhileBusy(objectPath);

        // Release the port; sensor recovers availability and its value.
        lock.reset();
        EXPECT_TRUE(co_await waitForAvailability(objectPath, true));
        EXPECT_TRUE(co_await waitForValue(objectPath, expectedValue));

        co_await stopDevice(*device);

        ctx.request_stop();
        co_return;
    }

    // A register with a poll-rate override is polled on its own schedule,
    // independent of the device poll rate.
    auto testRegisterPollRate(ProfileIntf::SensorRegister fastRegister,
                              ProfileIntf::SensorRegister slowRegister)
        -> sdbusplus::async::task<void>
    {
        EventIntf::Events events{ctx};
        // The device rate (1s from createDevice) drives fastRegister; the slow
        // register is overridden to a long interval so it is polled just once.
        std::unordered_map<std::string, std::chrono::seconds> overrides = {
            {slowRegister.name, 100s}};
        auto devPair =
            createDevice({fastRegister, slowRegister}, events, overrides);
        auto& device = devPair.second;

        ctx.spawn(device->pollRegisters());

        // fastRegister keeps being polled while slowRegister stays at one read.
        EXPECT_TRUE(co_await waitForReadCount(fastRegister.offset, 2));
        EXPECT_EQ(serverTester->readCount(slowRegister.offset), 1U);

        co_await stopDevice(*device);
        ctx.request_stop();
        co_return;
    }
};

TEST_F(SensorLifecycleTest, TestPortBusyMarksSensorUnavailable)
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
        .format = ProfileIntf::SensorFormat::fixedPoint,
    };

    ctx.spawn(testPortBusy(objectPath, sensorRegister,
                           TestIntf::testReadHoldingRegisterTempUnsigned[0]));

    ctx.run();
}

TEST_F(SensorLifecycleTest, TestRegisterPollRateOverride)
{
    setupDevice({
        "ResorviorPumpUnit",
        "xyz/openbmc_project/Inventory/ResorviorPumpUnit",
        ProfileIntf::DeviceType::reservoirPumpUnit,
        ProfileIntf::DeviceModel::DeltaRDF040DSS5193E0,
    });

    const ProfileIntf::SensorRegister fastRegister = {
        .name = "FastSensor",
        .type = SensorTypeIntf::temperature,
        .offset = TestIntf::testReadHoldingRegisterTempUnsignedOffset,
        .size = TestIntf::testReadHoldingRegisterTempCount,
        .format = ProfileIntf::SensorFormat::fixedPoint,
    };

    const ProfileIntf::SensorRegister slowRegister = {
        .name = "SlowSensor",
        .type = SensorTypeIntf::pressure,
        .offset = TestIntf::testReadHoldingRegisterDistantOffset,
        .size = TestIntf::testReadHoldingRegisterDistantCount,
        .format = ProfileIntf::SensorFormat::fixedPoint,
    };

    ctx.spawn(testRegisterPollRate(fastRegister, slowRegister));

    ctx.run();
}

// Verify that calling requestStop() causes the sensor polling coroutine to
// exit, sets isStopped() to true, and no further sensor polls occur.
TEST_F(SensorLifecycleTest, TestStopDeviceExitsAndStopsPolling)
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
        .format = ProfileIntf::SensorFormat::fixedPoint,
    };

    EventIntf::Events events{ctx};
    auto devPair = createDevice({sensorRegister}, events);
    auto& device = devPair.second;

    EXPECT_FALSE(device->isStopped()) << "Device should not be stopped yet";

    ctx.spawn(device->pollRegisters());

    // Let it poll once, then stop it
    ctx.spawn(
        sdbusplus::async::sleep_for(ctx, 800ms) |
        sdbusplus::async::execution::then([&]() { device->requestStop(); }));

    // Wait for coroutine to exit, then verify no further polls
    ctx.spawn(sdbusplus::async::sleep_for(ctx, 2s) |
              sdbusplus::async::execution::then([&]() {
                  EXPECT_TRUE(device->isStopped())
                      << "Device should be stopped after requestStop";
                  auto countAfterStop = serverTester->totalRequestCount.load();

                  // Schedule another check — count should not increase
                  ctx.spawn(
                      sdbusplus::async::sleep_for(ctx, 1s) |
                      sdbusplus::async::execution::then([&, countAfterStop]() {
                          EXPECT_EQ(serverTester->totalRequestCount.load(),
                                    countAfterStop)
                              << "No further polls after stop";
                          ctx.request_stop();
                      }));
              }));

    ctx.run();
}
