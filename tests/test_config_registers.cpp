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
        "PowerSupplyUnit",
        "xyz/openbmc_project/Inventory/PowerSupplyUnit",
        ProfileIntf::DeviceType::powerSupplyUnit,
        ProfileIntf::DeviceModel::Artesyn7000552480000,
    });

    testProfile.configRegisters = {
        {.name = "SyncTime",
         .type = ProfileIntf::ConfigType::unixTime,
         .offset = TestIntf::testConfigWriteRegisterOffset,
         .size = TestIntf::testConfigWriteRegisterCount,
         .period = 1}};

    auto testPeriodicWrite = [&]() -> sdbusplus::async::task<void> {
        EventIntf::Events events{ctx, stateDir};
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
        "PowerSupplyUnit",
        "xyz/openbmc_project/Inventory/PowerSupplyUnit",
        ProfileIntf::DeviceType::powerSupplyUnit,
        ProfileIntf::DeviceModel::Artesyn7000552480000,
    });

    // No Period -> written exactly once on bring-up, never rewritten.
    testProfile.configRegisters = {
        {.name = "SyncTime",
         .type = ProfileIntf::ConfigType::unixTime,
         .offset = TestIntf::testConfigWriteRegisterOffset,
         .size = TestIntf::testConfigWriteRegisterCount,
         .period = std::nullopt}};

    auto testOneShotWrite = [&]() -> sdbusplus::async::task<void> {
        EventIntf::Events events{ctx, stateDir};
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

TEST_F(ConfigRegistersTest, TestInitConfigRegisterWritesDefault)
{
    setupDevice({
        "BatteryBackupUnit",
        "xyz/openbmc_project/Inventory/BatteryBackupUnit",
        ProfileIntf::DeviceType::batteryBackupUnit,
        ProfileIntf::DeviceModel::DeltaBBUBC100AE000,
    });

    // Init register carries a profile default and is written once on bring-up.
    testProfile.configRegisters = {
        {.name = "BBU_1_2_DischargeTime",
         .type = ProfileIntf::ConfigType::init,
         .offset = TestIntf::testConfigWriteRegisterOffset,
         .size = 1,
         .period = std::nullopt,
         .defaultValue = {90}}};

    auto testInitWrite = [&]() -> sdbusplus::async::task<void> {
        EventIntf::Events events{ctx, stateDir};
        auto devPair = createDevice({}, events);
        auto& device = devPair.second;
        auto countBefore = serverTester->writeRequestCount.load();
        co_await device->pollRegisters();
        EXPECT_EQ(serverTester->writeRequestCount.load() - countBefore, 1U)
            << "Init config register should be written exactly once";
        co_return;
    };

    ctx.spawn(testInitWrite());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
