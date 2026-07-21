#pragma once

#include "common/events.hpp"
#include "device/device_factory.hpp"
#include "device/device_utils.hpp"
#include "modbus_rtu_config.hpp"
#include "port/base_port.hpp"
#include "test_base.hpp"

#include <xyz/openbmc_project/Association/Definitions/client.hpp>
#include <xyz/openbmc_project/Sensor/Value/client.hpp>
#include <xyz/openbmc_project/State/Decorator/Availability/client.hpp>
#include <xyz/openbmc_project/State/Decorator/OperationalStatus/client.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

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
using DeviceIntf::DeviceFactory;
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

// Shared base fixture for the sensor test suites. Each concrete suite passes
// its own D-Bus service name and socat path prefixes so the split binaries can
// run in parallel without colliding on the well-known name or PTY paths.
class SensorTestBase : public BaseTest
{
  public:
    static constexpr auto portName = "TestPort0";
    static constexpr auto sensorName = "OutletTemperature";

    // Polling for a D-Bus property to reach an expected state.
    static constexpr int propertyWaitRetries = 50;
    static constexpr auto propertyWaitInterval = std::chrono::milliseconds(100);

    std::string serviceName;
    PortConfigIntf::Config portConfig;
    DeviceTestConfig deviceTestConfig;
    std::string deviceName;
    std::string fullSensorName;
    std::string objectPath;

    SensorTestBase(const char* clientPathPrefix, const char* serverPathPrefix,
                   const char* service);

    void setupDevice(const DeviceTestConfig& config);

    auto getSensorObjectPath(const std::string& name, SensorTypeIntf sensorType)
        -> std::string;

    auto checkInventoryAssociations(const std::string& parentInventoryPath,
                                    const std::string& inventoryPath)
        -> sdbusplus::async::task<void>;

    auto createDevice(std::vector<ProfileIntf::SensorRegister> sensorRegisters,
                      EventIntf::Events& events,
                      std::unordered_map<std::string, std::chrono::seconds>
                          registerPollRates = {})
        -> std::pair<std::unique_ptr<MockPort>,
                     std::unique_ptr<DeviceIntf::BaseDevice>>;

    auto waitForAvailability(const std::string& path, bool want)
        -> sdbusplus::async::task<bool>;

    auto waitForValue(const std::string& path, double want)
        -> sdbusplus::async::task<bool>;

    auto waitForReadCount(uint16_t offset, uint32_t want)
        -> sdbusplus::async::task<bool>;

    auto stopDevice(DeviceIntf::BaseDevice& device)
        -> sdbusplus::async::task<void>;

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
