#include "test_sensor_base.hpp"

SensorTestBase::SensorTestBase(const char* clientPathPrefix,
                               const char* serverPathPrefix,
                               const char* service) :
    BaseTest(clientPathPrefix, serverPathPrefix, service), serviceName(service)
{
    portConfig.name = portName;
    portConfig.portMode = PortConfigIntf::PortMode::rs485;
    portConfig.baudRate = baudRate;
    portConfig.rtsDelay = 1;
    portConfig.timeout = std::chrono::microseconds(300000);
}

void SensorTestBase::setupDevice(const DeviceTestConfig& config)
{
    deviceTestConfig = config;

    deviceName = std::format("{}_{}_{}", deviceTestConfig.devicePrefix,
                             TestIntf::testDeviceAddress, portName);

    fullSensorName = std::format("{}_{}", deviceName, sensorName);
    if constexpr (ModbusIntf::appendUnitSuffix)
    {
        fullSensorName +=
            DeviceIntf::getUnitSuffix(ProfileIntf::SensorType::temperature);
    }

    objectPath = std::format("{}/{}/{}", SensorValueIntf::namespace_path::value,
                             SensorValueIntf::namespace_path::temperature,
                             fullSensorName);
}

auto SensorTestBase::getSensorObjectPath(const std::string& name,
                                         SensorTypeIntf sensorType)
    -> std::string
{
    auto sensorName = std::format("{}_{}", deviceName, name);
    if constexpr (ModbusIntf::appendUnitSuffix)
    {
        sensorName += DeviceIntf::getUnitSuffix(sensorType);
    }
    return std::format("{}/{}/{}", SensorValueIntf::namespace_path::value,
                       DeviceIntf::getPathSuffix(sensorType), sensorName);
}

auto SensorTestBase::checkInventoryAssociations(
    const std::string& parentInventoryPath, const std::string& inventoryPath)
    -> sdbusplus::async::task<void>
{
    auto associationProperties =
        co_await AssociationIntf(ctx)
            .service(serviceName)
            .path(objectPath)
            .properties();

    using Association = std::tuple<std::string, std::string, std::string>;
    std::vector<Association> expected = {
        {"monitoring", "monitored_by", parentInventoryPath},
        {"inventory", "sensors", parentInventoryPath},
        {"inventory", "all_sensors", parentInventoryPath},
        {"monitoring", "monitored_by", inventoryPath},
        {"inventory", "sensors", inventoryPath},
        {"inventory", "all_sensors", inventoryPath},
    };

    EXPECT_EQ(associationProperties.associations.size(), expected.size());
    for (const auto& assoc : expected)
    {
        EXPECT_NE(std::find(associationProperties.associations.begin(),
                            associationProperties.associations.end(), assoc),
                  associationProperties.associations.end())
            << "Missing association: " << std::get<0>(assoc) << ", "
            << std::get<1>(assoc) << ", " << std::get<2>(assoc);
    }
}

auto SensorTestBase::createDevice(
    std::vector<ProfileIntf::SensorRegister> sensorRegisters,
    EventIntf::Events& events,
    std::unordered_map<std::string, std::chrono::seconds> registerPollRates)
    -> std::pair<std::unique_ptr<MockPort>,
                 std::unique_ptr<DeviceIntf::BaseDevice>>
{
    testProfile.sensorRegisters = std::move(sensorRegisters);

    auto inventoryPath = sdbusplus::object_path(
        std::string(DeviceFactory::chassisInventoryPath) + "/" + deviceName);

    DeviceConfigIntf::DeviceFactoryConfig deviceFactoryConfig = {
        {
            .name = deviceName,
            .type = "TestDevice",
            .address = TestIntf::testDeviceAddress,
            .serialPort = portConfig.name,
            .parentInventoryPath =
                sdbusplus::object_path(deviceTestConfig.inventoryPath),
            .inventoryPath = std::move(inventoryPath),
            .profile = testProfile,
            .pollRate = 1s,
            .registerPollRates = std::move(registerPollRates),
        },
        deviceTestConfig.deviceType,
        deviceTestConfig.deviceModel,
    };
    auto mockPort =
        std::make_unique<MockPort>(ctx, portConfig, clientDevicePath);
    auto device = DeviceIntf::DeviceFactory::create(ctx, deviceFactoryConfig,
                                                    *mockPort, events);
    return {std::move(mockPort), std::move(device)};
}

auto SensorTestBase::waitForAvailability(const std::string& path, bool want)
    -> sdbusplus::async::task<bool>
{
    for (int i = 0; i < propertyWaitRetries; i++)
    {
        auto props = co_await AvailabilityIntf(ctx)
                         .service(serviceName)
                         .path(path)
                         .properties();
        if (props.available == want)
        {
            co_return true;
        }
        co_await sdbusplus::async::sleep_for(ctx, propertyWaitInterval);
    }
    co_return false;
}

auto SensorTestBase::waitForValue(const std::string& path, double want)
    -> sdbusplus::async::task<bool>
{
    for (int i = 0; i < propertyWaitRetries; i++)
    {
        auto props = co_await SensorValueIntf(ctx)
                         .service(serviceName)
                         .path(path)
                         .properties();
        if (props.value == want)
        {
            co_return true;
        }
        co_await sdbusplus::async::sleep_for(ctx, propertyWaitInterval);
    }
    co_return false;
}

auto SensorTestBase::waitForReadCount(uint16_t offset, uint32_t want)
    -> sdbusplus::async::task<bool>
{
    for (int i = 0; i < propertyWaitRetries; i++)
    {
        if (serverTester->readCount(offset) >= want)
        {
            co_return true;
        }
        co_await sdbusplus::async::sleep_for(ctx, propertyWaitInterval);
    }
    co_return false;
}

auto SensorTestBase::stopDevice(DeviceIntf::BaseDevice& device)
    -> sdbusplus::async::task<void>
{
    device.requestStop();
    while (!device.isStopped())
    {
        co_await sdbusplus::async::sleep_for(ctx, propertyWaitInterval);
    }
    co_return;
}
