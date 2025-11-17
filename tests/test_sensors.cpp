#include "common/events.hpp"
#include "device/device_factory.hpp"
#include "modbus_server_tester.hpp"
#include "port/base_port.hpp"

#include <fcntl.h>

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

namespace TestIntf = phosphor::modbus::test;
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

class SensorsTest : public ::testing::Test
{
  public:
    PortConfigIntf::Config portConfig;
    static constexpr const char* clientDevicePath = "/tmp/ttySensorsTestPort0";
    static constexpr const char* serverDevicePath = "/tmp/ttySensorsTestPort1";
    static constexpr auto portName = "TestPort0";
    static constexpr auto baudRate = 115200;
    static constexpr const auto strBaudeRate = "b115200";
    std::string deviceName;
    std::string fullSensorName;
    std::string objectPath;
    static constexpr auto serviceName =
        "xyz.openbmc_project.TestModbusRTUSensors";
    static constexpr auto sensorName = "OutletTemperature";
    int socat_pid = -1;
    sdbusplus::async::context ctx;
    int fdClient = -1;
    std::unique_ptr<TestIntf::ServerTester> serverTester;
    int fdServer = -1;

    SensorsTest()
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

        serverTester = std::make_unique<TestIntf::ServerTester>(ctx, fdServer);
    }

    ~SensorsTest() noexcept override
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

    auto checkInventoryAssociations() -> sdbusplus::async::task<void>
    {
        constexpr auto numOfInventoryAssociations = 2;
        auto associationProperties =
            co_await AssociationIntf(ctx)
                .service(serviceName)
                .path(objectPath)
                .properties();
        EXPECT_EQ(associationProperties.associations.size(),
                  numOfInventoryAssociations);
    }

    auto testSensorCreation(std::string objectPath,
                            DeviceConfigIntf::SensorRegister sensorRegister,
                            double expectedValue)
        -> sdbusplus::async::task<void>
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
                .sensorRegisters = {sensorRegister},
                .statusRegisters = {},
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
        EXPECT_EQ(properties.value, expectedValue) << "Sensor value mismatch";
        EXPECT_EQ(properties.unit, sensorRegister.unit)
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

    void SetUp() override
    {
        // Process request for sensor poll
        ctx.spawn(serverTester->processRequests());
    }
};

TEST_F(SensorsTest, TestSensorValueUnsigned)
{
    const DeviceConfigIntf::SensorRegister sensorRegister = {
        .name = sensorName,
        .pathSuffix = SensorValueIntf::namespace_path::temperature,
        .unit = SensorValueIntf::Unit::DegreesC,
        .offset = TestIntf::testReadHoldingRegisterTempUnsignedOffset,
        .size = TestIntf::testReadHoldingRegisterTempCount,
        .format = DeviceConfigIntf::SensorFormat::floatingPoint,
    };

    ctx.spawn(
        testSensorCreation(objectPath, sensorRegister,
                           TestIntf::testReadHoldingRegisterTempUnsigned[0]));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(SensorsTest, TestSensorValueSigned)
{
    const DeviceConfigIntf::SensorRegister sensorRegister = {
        .name = sensorName,
        .pathSuffix = SensorValueIntf::namespace_path::temperature,
        .unit = SensorValueIntf::Unit::DegreesC,
        .offset = TestIntf::testReadHoldingRegisterTempSignedOffset,
        .size = TestIntf::testReadHoldingRegisterTempCount,
        .isSigned = true,
        .format = DeviceConfigIntf::SensorFormat::floatingPoint,
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

TEST_F(SensorsTest, TestSensorValueWithSettings)
{
    const DeviceConfigIntf::SensorRegister sensorRegister = {
        .name = sensorName,
        .pathSuffix = SensorValueIntf::namespace_path::temperature,
        .unit = SensorValueIntf::Unit::DegreesC,
        .offset = TestIntf::testReadHoldingRegisterTempUnsignedOffset,
        .size = TestIntf::testReadHoldingRegisterTempCount,
        .precision = 2,
        .scale = 0.1,
        .shift = 50,
        .format = DeviceConfigIntf::SensorFormat::floatingPoint,
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
