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
                 const DeviceConfigIntf::Config& config,
                 PortIntf::BasePort& serialPort) :
        DeviceIntf::DeviceFirmware(ctx, config, serialPort)
    {}

    auto getObjectPath() -> sdbusplus::message::object_path
    {
        return objectPath;
    }
};

class FirmwareTest : public BaseTest
{
  public:
    static constexpr auto clientDevicePath = "/tmp/ttyFirmwareTestPort0";
    static constexpr auto serverDevicePath = "/tmp/ttyFirmwareTestPort1";
    static constexpr auto portName = "TestPort0";
    static constexpr auto serviceName =
        "xyz.openbmc_project.TestModbusRTUFirmware";
    static constexpr auto firmwareName = "TestVersion";
    PortConfigIntf::Config portConfig;
    std::string deviceName;
    std::string objectPath;
    std::unique_ptr<MockPort> mockPort;

    FirmwareTest() : BaseTest(clientDevicePath, serverDevicePath, serviceName)
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

    auto testFirmwareVersion(
        std::string objectPath,
        DeviceConfigIntf::FirmwareRegister firmwareRegister,
        std::string expectedVersion) -> sdbusplus::async::task<void>
    {
        DeviceConfigIntf::DeviceFactoryConfig deviceFactoryConfig = {
            {
                .address = TestIntf::testDeviceAddress,
                .parity = ModbusIntf::Parity::none,
                .baudRate = baudRate,
                .name = deviceName,
                .portName = portConfig.name,
                .inventoryPath = sdbusplus::message::object_path(
                    "xyz/openbmc_project/Inventory/ResorviorPumpUnit"),
                .sensorRegisters = {},
                .statusRegisters = {},
                .firmwareRegisters = {firmwareRegister},
            },
            DeviceConfigIntf::DeviceType::reservoirPumpUnit,
            DeviceConfigIntf::DeviceModel::RDF040DSS5193E0,
        };

        auto deviceFirmware =
            std::make_unique<TestFirmware>(ctx, deviceFactoryConfig, *mockPort);

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
    const DeviceConfigIntf::FirmwareRegister firmwareRegister = {
        .name = "",
        .type = DeviceConfigIntf::FirmwareRegisterType::version,
        .offset = TestIntf::testReadHoldingRegisterFirmwareVersionOffset,
        .size = TestIntf::testReadHoldingRegisterFirmwareVersionCount};

    ctx.spawn(testFirmwareVersion(
        objectPath, firmwareRegister,
        TestIntf::testReadHoldingRegisterFirmwareVersionStr));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
