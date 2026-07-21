#include "test_sensor_base.hpp"

// Runtime config-register writes to the device (periodic and one-shot).
// Profile parsing of ConfigRegisters is covered in test_device_profile.
class ConfigRegistersTest : public SensorTestBase
{
  public:
    ConfigRegistersTest() :
        SensorTestBase("/tmp/ttyConfigRegistersPort0",
                       "/tmp/ttyConfigRegistersPort1",
                       "xyz.openbmc_project.TestModbusRTUConfigRegisters")
    {}
};

TEST_F(ConfigRegistersTest, TestConfigRegisterWritePeriodic)
{
    setupDevice({
        "ResorviorPumpUnit",
        "xyz/openbmc_project/Inventory/ResorviorPumpUnit",
        ProfileIntf::DeviceType::reservoirPumpUnit,
        ProfileIntf::DeviceModel::DeltaRDF040DSS5193E0,
    });

    testProfile.configRegisters = {
        {.name = "SyncTime",
         .type = ProfileIntf::ConfigType::unixTime,
         .offset = TestIntf::testConfigWriteRegisterOffset,
         .size = TestIntf::testConfigWriteRegisterCount,
         .period = 1}};

    auto testPeriodicWrite = [&]() -> sdbusplus::async::task<void> {
        EventIntf::Events events{ctx};
        auto devPair = createDevice({}, events);
        auto& device = devPair.second;
        auto countBefore = serverTester->writeRequestCount.load();
        co_await device->pollRegisters();
        EXPECT_GE(serverTester->writeRequestCount.load() - countBefore, 1U)
            << "Expected at least one config register write";
        co_return;
    };

    ctx.spawn(testPeriodicWrite());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(ConfigRegistersTest, TestConfigRegisterWriteOneShot)
{
    setupDevice({
        "ResorviorPumpUnit",
        "xyz/openbmc_project/Inventory/ResorviorPumpUnit",
        ProfileIntf::DeviceType::reservoirPumpUnit,
        ProfileIntf::DeviceModel::DeltaRDF040DSS5193E0,
    });

    // No Period -> written exactly once on bring-up, never rewritten.
    testProfile.configRegisters = {
        {.name = "SyncTime",
         .type = ProfileIntf::ConfigType::unixTime,
         .offset = TestIntf::testConfigWriteRegisterOffset,
         .size = TestIntf::testConfigWriteRegisterCount,
         .period = std::nullopt}};

    auto testOneShotWrite = [&]() -> sdbusplus::async::task<void> {
        EventIntf::Events events{ctx};
        auto devPair = createDevice({}, events);
        auto& device = devPair.second;
        auto countBefore = serverTester->writeRequestCount.load();
        co_await device->pollRegisters();
        EXPECT_EQ(serverTester->writeRequestCount.load() - countBefore, 1U)
            << "One-shot config register should be written exactly once";
        co_return;
    };

    ctx.spawn(testOneShotWrite());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
