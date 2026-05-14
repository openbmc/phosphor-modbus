#include "common/events.hpp"
#include "device/device_factory.hpp"
#include "modbus_rtu_config.hpp"
#include "port/base_port.hpp"
#include "test_base.hpp"

#include <xyz/openbmc_project/Association/Definitions/client.hpp>
#include <xyz/openbmc_project/Metric/Value/client.hpp>

#include <cmath>
#include <string>

#include <gtest/gtest.h>

using namespace std::literals;
using namespace testing;
using MetricValueIntf =
    sdbusplus::client::xyz::openbmc_project::metric::Value<>;
using AssociationIntf =
    sdbusplus::client::xyz::openbmc_project::association::Definitions<>;

namespace ModbusIntf = phosphor::modbus::rtu;
namespace ProfileIntf = phosphor::modbus::rtu::profile;
namespace PortIntf = phosphor::modbus::rtu::port;
namespace PortConfigIntf = PortIntf::config;
namespace DeviceIntf = phosphor::modbus::rtu::device;
namespace DeviceConfigIntf = DeviceIntf::config;
namespace EventIntf = phosphor::modbus::events;
using DeviceIntf::DeviceFactory;
using MetricTypeIntf = ProfileIntf::MetricType;

class MockPort : public PortIntf::BasePort
{
  public:
    MockPort(sdbusplus::async::context& ctx,
             const PortConfigIntf::Config& config,
             const std::string& devicePath) : BasePort(ctx, config, devicePath)
    {}
};

class MetricsTest : public BaseTest
{
  public:
    static constexpr auto clientPathPrefix = "/tmp/ttyMetricsTestPort0";
    static constexpr auto serverPathPrefix = "/tmp/ttyMetricsTestPort1";
    static constexpr auto portName = "TestPort0";
    static constexpr auto serviceName =
        "xyz.openbmc_project.TestModbusRTUMetrics";
    static constexpr auto metricName = "Minutes_Since_Fully_Closed";

    PortConfigIntf::Config portConfig;
    std::string deviceName;
    std::string fullMetricName;
    std::string objectPath;

    MetricsTest() : BaseTest(clientPathPrefix, serverPathPrefix, serviceName)
    {
        portConfig.name = portName;
        portConfig.portMode = PortConfigIntf::PortMode::rs485;
        portConfig.baudRate = baudRate;
        portConfig.rtsDelay = 1;

        deviceName = std::format("{}_{}_{}", "Valve",
                                 TestIntf::testDeviceAddress, portName);

        fullMetricName = std::format("{}_{}", deviceName, metricName);

        objectPath =
            std::format("{}/{}/{}", MetricValueIntf::namespace_path::value,
                        MetricValueIntf::namespace_path::valve_closed_duration,
                        fullMetricName);
    }

    auto getMetricObjectPath(const std::string& name, MetricTypeIntf metricType)
        -> std::string
    {
        return std::format(
            "{}/{}/{}_{}", MetricValueIntf::namespace_path::value,
            DeviceIntf::getMetricPathSuffix(metricType), deviceName, name);
    }

    auto createDevice(std::vector<ProfileIntf::MetricRegister> metricRegisters,
                      EventIntf::Events& events)
        -> std::pair<std::unique_ptr<MockPort>,
                     std::unique_ptr<DeviceIntf::BaseDevice>>
    {
        testProfile.metricRegisters = std::move(metricRegisters);

        auto inventoryPath = sdbusplus::object_path(
            std::string(DeviceFactory::chassisInventoryPath) + "/" +
            deviceName);

        DeviceConfigIntf::DeviceFactoryConfig deviceFactoryConfig = {
            {
                .name = deviceName,
                .type = "TestDevice",
                .address = TestIntf::testDeviceAddress,
                .serialPort = portConfig.name,
                .parentInventoryPath = sdbusplus::object_path(
                    "xyz/openbmc_project/Inventory/Valve"),
                .inventoryPath = std::move(inventoryPath),
                .profile = testProfile,
            },
            ProfileIntf::DeviceType::valve,
            ProfileIntf::DeviceModel::BelimoEV200ARXE,
        };
        auto mockPort =
            std::make_unique<MockPort>(ctx, portConfig, clientDevicePath);
        auto device = DeviceIntf::DeviceFactory::create(
            ctx, deviceFactoryConfig, *mockPort, events);
        return {std::move(mockPort), std::move(device)};
    }

    ProfileIntf::DeviceProfile testProfile = {
        .parity = ModbusIntf::Parity::none,
        .baudRate = baudRate,
        .probeRegister = {},
        .inventoryRegisters = {},
        .sensorRegisters = {},
        .statusRegisters = {},
        .metricRegisters = {},
        .firmwareRegisters = {},
    };
};

// Verify that a metric register is created with the correct value, unit, and
// associations on D-Bus after a single poll cycle.
TEST_F(MetricsTest, TestMetricValueUnsigned)
{
    const ProfileIntf::MetricRegister metricRegister = {
        .name = metricName,
        .type = MetricTypeIntf::valveClosedDuration,
        .offset = TestIntf::testReadHoldingRegisterMetricOffset,
        .size = TestIntf::testReadHoldingRegisterMetricCount,
        .scale = 60.0,
        .format = ProfileIntf::SensorFormat::floatingPoint,
        .pollInterval = 1s,
    };

    // Raw value 0x012C = 300, with scale=60 -> 300 * 60 = 18000 seconds
    constexpr double expectedValue = 300.0 * 60.0;

    auto testMetric = [&]() -> sdbusplus::async::task<void> {
        EventIntf::Events events{ctx};
        auto devPair = createDevice({metricRegister}, events);
        auto& device = devPair.second;

        co_await device->pollRegisters();

        auto properties = co_await MetricValueIntf(ctx)
                              .service(serviceName)
                              .path(objectPath)
                              .properties();
        EXPECT_EQ(properties.value, expectedValue) << "Metric value mismatch";
        EXPECT_EQ(properties.unit,
                  DeviceIntf::getMetricUnit(metricRegister.type))
            << "Metric unit mismatch";
        EXPECT_TRUE(std::isnan(properties.min_value)) << "Min value mismatch";
        EXPECT_TRUE(std::isnan(properties.max_value)) << "Max value mismatch";

        co_return;
    };

    ctx.spawn(testMetric());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 2s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

// Verify that metric associations use measuring/measured_by for both
// parent and device inventory paths.
TEST_F(MetricsTest, TestMetricAssociations)
{
    const ProfileIntf::MetricRegister metricRegister = {
        .name = metricName,
        .type = MetricTypeIntf::valveClosedDuration,
        .offset = TestIntf::testReadHoldingRegisterMetricOffset,
        .size = TestIntf::testReadHoldingRegisterMetricCount,
        .scale = 60.0,
        .format = ProfileIntf::SensorFormat::floatingPoint,
    };

    auto testAssociations = [&]() -> sdbusplus::async::task<void> {
        EventIntf::Events events{ctx};
        auto devPair = createDevice({metricRegister}, events);

        auto associationProperties =
            co_await AssociationIntf(ctx)
                .service(serviceName)
                .path(objectPath)
                .properties();

        auto inventoryPath = sdbusplus::object_path(
            std::string(DeviceFactory::chassisInventoryPath) + "/" +
            deviceName);

        using Association = std::tuple<std::string, std::string, std::string>;
        std::vector<Association> expected = {
            {"measuring", "measured_by", "xyz/openbmc_project/Inventory/Valve"},
            {"measuring", "measured_by", inventoryPath},
        };

        EXPECT_EQ(associationProperties.associations.size(), expected.size());
        for (const auto& assoc : expected)
        {
            EXPECT_NE(std::find(associationProperties.associations.begin(),
                                associationProperties.associations.end(),
                                assoc),
                      associationProperties.associations.end())
                << "Missing association: " << std::get<0>(assoc) << ", "
                << std::get<1>(assoc) << ", " << std::get<2>(assoc);
        }

        co_return;
    };

    ctx.spawn(testAssociations());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

// Verify that a metric with Integer format passes through the raw value
// without applying scale/shift/precision conversion.
TEST_F(MetricsTest, TestMetricValueInteger)
{
    const ProfileIntf::MetricRegister metricRegister = {
        .name = metricName,
        .type = MetricTypeIntf::valveClosedDuration,
        .offset = TestIntf::testReadHoldingRegisterMetricOffset,
        .size = TestIntf::testReadHoldingRegisterMetricCount,
        .scale = 60.0,
        .format = ProfileIntf::SensorFormat::integer,
    };

    // Raw value 0x012C = 300, Integer format ignores scale -> 300
    constexpr double expectedValue = 300.0;

    auto testMetric = [&]() -> sdbusplus::async::task<void> {
        EventIntf::Events events{ctx};
        auto devPair = createDevice({metricRegister}, events);
        auto& device = devPair.second;

        co_await device->pollRegisters();

        auto properties = co_await MetricValueIntf(ctx)
                              .service(serviceName)
                              .path(objectPath)
                              .properties();
        EXPECT_EQ(properties.value, expectedValue) << "Metric value mismatch";

        co_return;
    };

    ctx.spawn(testMetric());

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
