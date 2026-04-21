#include "device/device_factory.hpp"
#include "modbus_server_tester.hpp"
#include "port/base_port.hpp"
#include "test_base.hpp"

#include <xyz/openbmc_project/Software/Version/client.hpp>

#include <gtest/gtest.h>

using namespace std::literals;
using namespace testing;
using SoftwareIntf =
    sdbusplus::client::xyz::openbmc_project::software::Version<>;

namespace ModbusIntf = phosphor::modbus::rtu;
namespace ProfileIntf = phosphor::modbus::rtu::profile;
namespace PortIntf = phosphor::modbus::rtu::port;
namespace PortConfigIntf = PortIntf::config;
namespace DeviceIntf = phosphor::modbus::rtu::device;
namespace DeviceConfigIntf = DeviceIntf::config;

class MockPort : public PortIntf::BasePort
{
  public:
    MockPort(sdbusplus::async::context& ctx,
             const PortConfigIntf::Config& config,
             const std::string& devicePath) : BasePort(ctx, config, devicePath)
    {}
};

class TestFirmware : public DeviceIntf::DeviceFirmware
{
  public:
    TestFirmware(sdbusplus::async::context& ctx,
                 const ModbusIntf::config::Config& config,
                 PortIntf::BasePort& serialPort) :
        DeviceIntf::DeviceFirmware(ctx, config, serialPort)
    {}

    auto getObjectPath() -> sdbusplus::object_path
    {
        return objectPath;
    }
};

class FirmwareTest : public BaseTest
{
  public:
    static constexpr auto clientPathPrefix = "/tmp/ttyFirmwareTestPort0";
    static constexpr auto serverPathPrefix = "/tmp/ttyFirmwareTestPort1";
    static constexpr auto portName = "TestPort0";
    static constexpr auto serviceName =
        "xyz.openbmc_project.TestModbusRTUFirmware";
    static constexpr auto firmwareName = "TestVersion";
    PortConfigIntf::Config portConfig;
    std::string deviceName;
    std::string objectPath;
    std::unique_ptr<MockPort> mockPort;

    FirmwareTest() : BaseTest(clientPathPrefix, serverPathPrefix, serviceName)
    {
        portConfig.name = portName;
        portConfig.portMode = PortConfigIntf::PortMode::rs485;
        portConfig.baudRate = baudRate;
        portConfig.rtsDelay = 1;

        deviceName = std::format("ResorviorPumpUnit_{}_{}",
                                 TestIntf::testDeviceAddress, portName);
        objectPath =
            std::format("{}/{}", SoftwareIntf::namespace_path, deviceName);

        mockPort =
            std::make_unique<MockPort>(ctx, portConfig, clientDevicePath);
    }

    auto testFirmwareVersion(std::string objectPath,
                             ProfileIntf::FirmwareRegister firmwareRegister,
                             std::string expectedVersion)
        -> sdbusplus::async::task<void>
    {
        ProfileIntf::DeviceProfile testProfile = {
            .parity = ModbusIntf::Parity::none,
            .baudRate = baudRate,
            .probeRegister = {},
            .inventoryRegisters = {},
            .sensorRegisters = {},
            .statusRegisters = {},
            .firmwareRegisters = {firmwareRegister},
        };

        ModbusIntf::config::Config baseConfig = {
            .name = deviceName,
            .type = "TestDevice",
            .address = TestIntf::testDeviceAddress,
            .serialPort = portConfig.name,
            .inventoryPath = sdbusplus::object_path(
                "xyz/openbmc_project/Inventory/ResorviorPumpUnit"),
            .profile = testProfile,
        };

        auto deviceFirmware =
            std::make_unique<TestFirmware>(ctx, baseConfig, *mockPort);

        co_await deviceFirmware->readVersionRegister();

        EXPECT_TRUE(deviceFirmware->getObjectPath().str.starts_with(objectPath))
            << "Invalid ObjectPath";

        auto softwarePath = deviceFirmware->getObjectPath().str;

        auto properties = co_await SoftwareIntf(ctx)
                              .service(serviceName)
                              .path(softwarePath)
                              .properties();

        EXPECT_EQ(properties.version, expectedVersion)
            << "Firmware version mismatch";

        co_return;
    }
};

TEST_F(FirmwareTest, TestFirmwareVersion)
{
    const ProfileIntf::FirmwareRegister firmwareRegister = {
        .name = "",
        .type = ProfileIntf::FirmwareRegisterType::version,
        .offset = TestIntf::testReadHoldingRegisterFirmwareVersionOffset,
        .size = TestIntf::testReadHoldingRegisterFirmwareVersionCount};

    ctx.spawn(testFirmwareVersion(
        objectPath, firmwareRegister,
        TestIntf::testReadHoldingRegisterFirmwareVersionStr));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
