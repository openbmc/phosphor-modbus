#include "device/device_factory.hpp"
#include "modbus_server_tester.hpp"
#include "port/base_port.hpp"

#include <fcntl.h>

#include <xyz/openbmc_project/Software/Version/client.hpp>

#include <gtest/gtest.h>

using namespace std::literals;
using namespace testing;
using SoftwareIntf =
    sdbusplus::client::xyz::openbmc_project::software::Version<>;

namespace TestIntf = phosphor::modbus::test;
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

class FirmwareTest : public ::testing::Test
{
  public:
    PortConfigIntf::Config portConfig;
    static constexpr const char* clientDevicePath = "/tmp/ttyFirmwareTestPort0";
    static constexpr const char* serverDevicePath = "/tmp/ttyFirmwareTestPort1";
    static constexpr auto portName = "TestPort0";
    static constexpr auto baudRate = 115200;
    static constexpr const auto strBaudeRate = "b115200";
    std::string deviceName;
    std::string objectPath;
    static constexpr auto serviceName =
        "xyz.openbmc_project.TestModbusRTUFirmware";
    static constexpr auto firmwareName = "TestVersion";
    int socat_pid = -1;
    sdbusplus::async::context ctx;
    int fdClient = -1;
    std::unique_ptr<TestIntf::ServerTester> serverTester;
    int fdServer = -1;
    std::unique_ptr<MockPort> mockPort;

    FirmwareTest()
    {
        portConfig.name = portName;
        portConfig.portMode = PortConfigIntf::PortMode::rs485;
        portConfig.baudRate = baudRate;
        portConfig.rtsDelay = 1;

        deviceName = std::format("ResorviorPumpUnit_{}_{}",
                                 TestIntf::testDeviceAddress, portName);
        objectPath =
            std::format("{}/{}", SoftwareIntf::namespace_path, deviceName);

        std::string socatCmd = std::format(
            "socat -x -v -d -d pty,link={},rawer,echo=0,parenb,{} pty,link={},rawer,echo=0,parenb,{} & echo $!",
            serverDevicePath, strBaudeRate, clientDevicePath, strBaudeRate);

        // Start socat in the background and capture its PID
        FILE* fp = popen(socatCmd.c_str(), "r");
        EXPECT_NE(fp, nullptr) << "Failed to start socat: " << strerror(errno);
        EXPECT_GT(fscanf(fp, "%d", &socat_pid), 0);
        pclose(fp);

        // Wait for socat to start up
        sleep(1);

        fdClient = open(clientDevicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
        EXPECT_NE(fdClient, -1)
            << "Failed to open serial port " << clientDevicePath
            << " with error: " << strerror(errno);

        fdServer = open(serverDevicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
        EXPECT_NE(fdServer, -1)
            << "Failed to open serial port " << serverDevicePath
            << " with error: " << strerror(errno);

        ctx.request_name(serviceName);

        mockPort =
            std::make_unique<MockPort>(ctx, portConfig, clientDevicePath);

        serverTester = std::make_unique<TestIntf::ServerTester>(ctx, fdServer);
    }

    ~FirmwareTest() noexcept override
    {
        if (fdClient != -1)
        {
            close(fdClient);
            fdClient = -1;
        }
        if (fdServer != -1)
        {
            close(fdServer);
            fdServer = -1;
        }
        kill(socat_pid, SIGTERM);
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

    void SetUp() override
    {
        // Process request to read firmware version
        ctx.spawn(serverTester->processRequests());
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
