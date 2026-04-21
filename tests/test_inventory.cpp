#include "inventory/modbus_inventory.hpp"
#include "modbus_server_tester.hpp"
#include "port/base_port.hpp"
#include "test_base.hpp"

#include <xyz/openbmc_project/Inventory/Decorator/Asset/client.hpp>
#include <xyz/openbmc_project/Inventory/Item/Chassis/client.hpp>

#include <gtest/gtest.h>

using namespace std::literals;
using namespace testing;
using AssetClientIntf =
    sdbusplus::client::xyz::openbmc_project::inventory::decorator::Asset<>;
using ChassisClientIntf =
    sdbusplus::client::xyz::openbmc_project::inventory::item::Chassis<>;

namespace ModbusIntf = phosphor::modbus::rtu;
namespace ProfileIntf = phosphor::modbus::rtu::profile;
namespace PortIntf = phosphor::modbus::rtu::port;
namespace PortConfigIntf = PortIntf::config;
namespace InventoryIntf = phosphor::modbus::rtu::inventory;

class MockPort : public PortIntf::BasePort
{
  public:
    MockPort(sdbusplus::async::context& ctx,
             const PortConfigIntf::Config& config,
             const std::string& devicePath) : BasePort(ctx, config, devicePath)
    {}
};

class InventoryTest : public BaseTest
{
  public:
    static constexpr const char* clientPathPrefix =
        "/tmp/ttyInventoryTestPort0";
    static constexpr const char* serverPathPrefix =
        "/tmp/ttyInventoryTestPort1";
    static constexpr const auto deviceName = "Test1";
    static constexpr auto serviceName = "xyz.openbmc_project.TestModbusRTU";
    PortConfigIntf::Config portConfig;

    InventoryTest() : BaseTest(clientPathPrefix, serverPathPrefix, serviceName)
    {
        portConfig.name = "TestPort1";
        portConfig.portMode = PortConfigIntf::PortMode::rs485;
        portConfig.baudRate = 115200;
        portConfig.rtsDelay = 1;
    }

    auto createDevice(
        const ProfileIntf::DeviceProfile& profile,
        std::chrono::seconds dormantPeriod = std::chrono::seconds(0))
        -> std::pair<std::unique_ptr<MockPort>,
                     std::unique_ptr<InventoryIntf::Device>>
    {
        ModbusIntf::config::Config baseConfig = {
            .name = deviceName,
            .type = "TestDevice",
            .address = TestIntf::testDeviceAddress,
            .serialPort = portConfig.name,
            .inventoryPath = sdbusplus::object_path("/"),
            .profile = profile,
        };

        auto mockPort =
            std::make_unique<MockPort>(ctx, portConfig, clientDevicePath);

        auto device = std::make_unique<InventoryIntf::Device>(
            ctx, baseConfig, *mockPort, nullptr, dormantPeriod);

        return {std::move(mockPort), std::move(device)};
    }

    // Profile with a valid register for successful probes
    ProfileIntf::DeviceProfile testProfile = {
        .probeRegister = {.offset =
                              TestIntf::testReadHoldingRegisterModelOffset,
                          .size = TestIntf::testReadHoldingRegisterModelCount,
                          .expectedValue = std::string("RDF040DSS5190")},
        .parity = ModbusIntf::Parity::none,
        .baudRate = 115200,
        .inventoryRegisters =
            {{.name = "Model",
              .type = ProfileIntf::InventoryDataType::model,
              .offset = TestIntf::testReadHoldingRegisterModelOffset,
              .size = TestIntf::testReadHoldingRegisterModelCount}},
        .sensorRegisters = {},
        .statusRegisters = {},
        .firmwareRegisters = {},
    };

    // Profile with a register that causes probe failure
    ProfileIntf::DeviceProfile failProfile = {
        .probeRegister = {.offset = TestIntf::testFailureReadHoldingRegister,
                          .size = 0x1,
                          .expectedValue = uint64_t(0)},
        .parity = ModbusIntf::Parity::none,
        .baudRate = 115200,
        .inventoryRegisters = {{.name = "Unknown",
                                .type = ProfileIntf::InventoryDataType::unknown,
                                .offset =
                                    TestIntf::testFailureReadHoldingRegister,
                                .size = 0x1}},
        .sensorRegisters = {},
        .statusRegisters = {},
        .firmwareRegisters = {},
    };

    // Profile with a flaky register that alternates success/failure
    ProfileIntf::DeviceProfile flakyProfile = {
        .probeRegister = {.offset =
                              TestIntf::testFlakyReadHoldingRegisterOffset,
                          .size = TestIntf::testFlakyReadHoldingRegisterCount,
                          .expectedValue = uint64_t(0x0050)},
        .parity = ModbusIntf::Parity::none,
        .baudRate = 115200,
        .inventoryRegisters =
            {{.name = "Flaky",
              .type = ProfileIntf::InventoryDataType::unknown,
              .offset = TestIntf::testFlakyReadHoldingRegisterOffset,
              .size = TestIntf::testFlakyReadHoldingRegisterCount}},
        .sensorRegisters = {},
        .statusRegisters = {},
        .firmwareRegisters = {},
    };

    // Profile with a null-padded string probe value: "AB" with trailing nulls
    ProfileIntf::DeviceProfile nullPaddedProfile = {
        .probeRegister =
            {.offset = TestIntf::testReadHoldingRegisterNullPaddedOffset,
             .size = TestIntf::testReadHoldingRegisterNullPaddedCount,
             .expectedValue = std::string("AB")},
        .parity = ModbusIntf::Parity::none,
        .baudRate = 115200,
        .inventoryRegisters = {},
        .sensorRegisters = {},
        .statusRegisters = {},
        .firmwareRegisters = {},
    };

    // Profile where the read succeeds but value doesn't match
    ProfileIntf::DeviceProfile mismatchProfile = {
        .probeRegister = {.offset =
                              TestIntf::testReadHoldingRegisterModelOffset,
                          .size = TestIntf::testReadHoldingRegisterModelCount,
                          .expectedValue = std::string("XXXXXX")},
        .parity = ModbusIntf::Parity::none,
        .baudRate = 115200,
        .inventoryRegisters = {},
        .sensorRegisters = {},
        .statusRegisters = {},
        .firmwareRegisters = {},
    };

    auto testInventoryObjectCreation(std::string objPath)
        -> sdbusplus::async::task<void>
    {
        auto devicePair = createDevice(testProfile);
        auto& inventoryDevice = devicePair.second;

        co_await inventoryDevice->startProbing();

        // Read back Asset properties from D-Bus
        auto assetProps = co_await AssetClientIntf(ctx)
                              .service(serviceName)
                              .path(objPath)
                              .properties();

        EXPECT_EQ(assetProps.model, TestIntf::testReadHoldingRegisterModelStr)
            << "Model mismatch";
        EXPECT_EQ(assetProps.serial_number, "") << "Serial number mismatch";

        // Read back Chassis properties from D-Bus
        auto chassisProps = co_await ChassisClientIntf(ctx)
                                .service(serviceName)
                                .path(objPath)
                                .properties();

        using ChassisType = ChassisClientIntf::ChassisType;
        EXPECT_EQ(chassisProps.type, ChassisType::Module)
            << "Chassis type mismatch";

        co_return;
    }

    auto testDormantSkippedAndExpires() -> sdbusplus::async::task<void>
    {
        constexpr auto testDormantPeriod = std::chrono::seconds(1);

        auto devicePair = createDevice(failProfile, testDormantPeriod);
        auto& inventoryDevice = devicePair.second;

        // Device should not be dormant initially
        EXPECT_FALSE(inventoryDevice->isDormant());

        // First probe fails, device should be marked dormant
        co_await inventoryDevice->probeDevice();
        EXPECT_TRUE(inventoryDevice->isDormant())
            << "Device should be dormant after failed probe";

        // Second probe should skip (device is dormant, no server request)
        auto countBeforeDormantProbe = serverTester->totalRequestCount.load();
        co_await inventoryDevice->probeDevice();
        EXPECT_TRUE(inventoryDevice->isDormant())
            << "Device should still be dormant";
        EXPECT_EQ(serverTester->totalRequestCount.load(),
                  countBeforeDormantProbe)
            << "Server should not receive a request for dormant device";

        // Wait for dormant period to expire
        co_await sdbusplus::async::sleep_for(
            ctx, testDormantPeriod + std::chrono::seconds(1));

        // Third probe should clear dormant state and re-probe
        auto countBeforeReprobe = serverTester->totalRequestCount.load();
        co_await inventoryDevice->probeDevice();
        // Device gets re-probed, fails again, so it's dormant again
        EXPECT_TRUE(inventoryDevice->isDormant())
            << "Device should be dormant again after re-probe failure";
        EXPECT_GT(serverTester->totalRequestCount.load(), countBeforeReprobe)
            << "Server should receive a request after dormant period expires";

        co_return;
    }
};

TEST_F(InventoryTest, TestAddInventoryObject)
{
    auto objPath =
        std::format("{}/{}_{}_{}", InventoryIntf::Device::inventoryServerPath,
                    deviceName, TestIntf::testDeviceAddress, portConfig.name);

    ctx.spawn(testInventoryObjectCreation(objPath));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

// Verify that a failed probe does not create an inventory object on D-Bus.
TEST_F(InventoryTest, TestNonRespondingAddressNoInventoryObject)
{
    auto testProbe = [&]() -> sdbusplus::async::task<void> {
        auto devicePair = createDevice(failProfile);
        auto& inventoryDevice = devicePair.second;

        co_await inventoryDevice->startProbing();

        auto objPath = std::format(
            "{}/{}_{}_{}", InventoryIntf::Device::inventoryServerPath,
            deviceName, TestIntf::testDeviceAddress, portConfig.name);

        bool exists = false;
        try
        {
            co_await AssetClientIntf(ctx)
                .service(serviceName)
                .path(objPath)
                .properties();
            exists = true;
        }
        catch (...)
        {
            exists = false;
        }

        EXPECT_FALSE(exists)
            << "Inventory object should not exist for failed probe";

        co_return;
    };

    ctx.spawn(testProbe());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 3s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

// Verify the dormant state machine: a failed probe for an undiscovered address
// marks it dormant, subsequent probes skip it without hitting the bus, and
// after the dormant period expires the address becomes eligible for probing
// again.
TEST_F(InventoryTest, TestDormantDeviceSkippedAndExpires)
{
    ctx.spawn(testDormantSkippedAndExpires());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 5s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

// Verify that a previously discovered device is never marked dormant. Uses a
// flaky register that alternates success/failure to simulate a device that
// goes offline and comes back. The device should be removed on failure but
// remain eligible for immediate re-probing (not dormant).
TEST_F(InventoryTest, TestDiscoveredDeviceNotMarkedDormant)
{
    auto testProbe = [&]() -> sdbusplus::async::task<void> {
        auto devicePair = createDevice(flakyProfile);
        auto& inventoryDevice = devicePair.second;

        // 1st probe succeeds - device discovered
        co_await inventoryDevice->probeDevice();
        EXPECT_FALSE(inventoryDevice->isDormant())
            << "Device should not be dormant after successful probe";

        // 2nd probe fails - device removed but NOT dormant
        co_await inventoryDevice->probeDevice();
        EXPECT_FALSE(inventoryDevice->isDormant())
            << "Previously discovered device should not be marked dormant";

        // 3rd probe succeeds - device re-discovered
        co_await inventoryDevice->probeDevice();
        EXPECT_FALSE(inventoryDevice->isDormant())
            << "Device should not be dormant after re-discovery";

        co_return;
    };

    ctx.spawn(testProbe());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 3s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

// Verify that a successful register read with a mismatched probe value
// does not create an inventory object.
TEST_F(InventoryTest, TestProbeValueMismatchNoInventoryObject)
{
    auto testProbe = [&]() -> sdbusplus::async::task<void> {
        auto devicePair = createDevice(mismatchProfile);
        auto& inventoryDevice = devicePair.second;

        co_await inventoryDevice->probeDevice();

        auto objPath = std::format(
            "{}/{}_{}_{}", InventoryIntf::Device::inventoryServerPath,
            deviceName, TestIntf::testDeviceAddress, portConfig.name);

        bool exists = false;
        try
        {
            co_await AssetClientIntf(ctx)
                .service(serviceName)
                .path(objPath)
                .properties();
            exists = true;
        }
        catch (...)
        {
            exists = false;
        }

        EXPECT_FALSE(exists)
            << "Inventory object should not exist when probe value mismatches";

        co_return;
    };

    ctx.spawn(testProbe());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 3s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

// Verify that probe matching correctly strips multiple trailing null bytes
// from register data when comparing against the expected string value.
TEST_F(InventoryTest, TestProbeNullPaddedStringMatch)
{
    auto testProbe = [&]() -> sdbusplus::async::task<void> {
        auto devicePair = createDevice(nullPaddedProfile);
        auto& inventoryDevice = devicePair.second;

        co_await inventoryDevice->probeDevice();

        // Device should not be dormant — probe value "AB" should match
        // after stripping nulls from {0x4142, 0x0000, 0x0000, 0x0000}
        EXPECT_FALSE(inventoryDevice->isDormant())
            << "Null-padded probe should match after stripping nulls";

        co_return;
    };

    ctx.spawn(testProbe());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 3s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
