#include "device_profile.hpp"

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

using namespace phosphor::modbus::rtu;
using namespace phosphor::modbus::rtu::profile;

static constexpr auto testProfileName = "TestProfile";
static constexpr auto testProfileJson = R"({
    "DeviceType": "ReservoirPumpUnit",
    "DeviceModel": "DeltaRDF040DSS5193E0",
    "Parity": "Even",
    "BaudRate": 9600,
    "ProbeRegister": {
        "Offset": "0x32",
        "Size": 2,
        "ExpectedValue": "TestDevice"
    },
    "InventoryRegisters": [
        {
            "Type": "Model",
            "Offset": "0x64",
            "Size": 4
        },
        {
            "Type": "SerialNumber",
            "Offset": "0xC8",
            "Size": 8
        }
    ],
    "SensorRegisters": [
        {
            "Name": "TestTemp",
            "Type": "Temperature",
            "Offset": "0x12C",
            "Size": 1,
            "Precision": 2,
            "Scale": 0.5,
            "Shift": 10.0,
            "IsSigned": true,
            "Format": "FixedPoint"
        },
        {
            "Name": "TestVoltage",
            "Type": "Voltage",
            "Offset": "0x190",
            "Size": 2,
            "Format": "Integer"
        }
    ],
    "StatusRegisters": [
        {
            "Offset": "0x1F4",
            "Bits": [
                {
                    "Name": "TestTemp",
                    "Type": "SensorFailure",
                    "BitPosition": 0,
                    "Value": true
                },
                {
                    "Name": "TestVoltage",
                    "Type": "SensorReadingCritical",
                    "BitPosition": 3,
                    "Value": false
                }
            ]
        },
        {
            "Name": "RPU",
            "Offset": "0x1F5",
            "Bits": [
                {
                    "Name": "PumpFault",
                    "Type": "PumpFailure",
                    "BitPosition": 1,
                    "Value": true
                }
            ]
        }
    ],
    "MetricRegisters": [
        {
            "Name": "TestClosedDuration",
            "Type": "ValveClosedDuration",
            "Offset": "0x2BC",
            "Size": 2,
            "Precision": 0,
            "Scale": 60.0,
            "Shift": 0.0,
            "IsSigned": true,
            "Format": "FixedPoint"
        }
    ],
    "FirmwareRegisters": [
        {
            "Name": "TestFW",
            "Type": "Version",
            "Offset": "0x258",
            "Size": 2
        }
    ],
    "ConfigRegisters": [
        {
            "Name": "TestSyncTime",
            "Type": "UnixTime",
            "Offset": "0x320",
            "Size": 2,
            "Period": 3600
        },
        {
            "Type": "UnixTime",
            "Offset": "0x321",
            "Size": 2
        }
    ]
})";

class DeviceProfileTest : public ::testing::Test
{
  protected:
    std::filesystem::path testProfilePath =
        std::filesystem::path(PROFILE_DIR) /
        (std::string(testProfileName) + ".json");

    void SetUp() override
    {
        std::ofstream(testProfilePath) << testProfileJson;
    }

    void TearDown() override
    {
        std::filesystem::remove(testProfilePath);
    }
};

TEST_F(DeviceProfileTest, ParsesAllRegisterTypes)
{
    const auto& profile = getDeviceProfile(testProfileName);

    EXPECT_EQ(profile.parity, Parity::even);
    EXPECT_EQ(profile.baudRate, 9600U);

    // Inventory registers
    ASSERT_EQ(profile.inventoryRegisters.size(), 2U);
    EXPECT_EQ(profile.inventoryRegisters[0].type, InventoryDataType::model);
    EXPECT_EQ(profile.inventoryRegisters[0].offset, 100U);
    EXPECT_EQ(profile.inventoryRegisters[0].size, 4U);
    EXPECT_EQ(profile.inventoryRegisters[1].type,
              InventoryDataType::serialNumber);
    EXPECT_EQ(profile.inventoryRegisters[1].offset, 200U);
    EXPECT_EQ(profile.inventoryRegisters[1].size, 8U);

    // Sensor registers — first has explicit optional fields, second uses
    // defaults
    ASSERT_EQ(profile.sensorRegisters.size(), 2U);

    const auto& temp = profile.sensorRegisters[0];
    EXPECT_EQ(temp.name, "TestTemp");
    EXPECT_EQ(temp.type, SensorType::temperature);
    EXPECT_EQ(temp.offset, 300U);
    EXPECT_EQ(temp.size, 1U);
    EXPECT_EQ(temp.precision, 2U);
    EXPECT_DOUBLE_EQ(temp.scale, 0.5);
    EXPECT_DOUBLE_EQ(temp.shift, 10.0);
    EXPECT_TRUE(temp.isSigned);
    EXPECT_EQ(temp.format, SensorFormat::fixedPoint);

    const auto& voltage = profile.sensorRegisters[1];
    EXPECT_EQ(voltage.name, "TestVoltage");
    EXPECT_EQ(voltage.type, SensorType::voltage);
    EXPECT_EQ(voltage.offset, 400U);
    EXPECT_EQ(voltage.size, 2U);
    EXPECT_EQ(voltage.precision, 0U);
    EXPECT_DOUBLE_EQ(voltage.scale, 1.0);
    EXPECT_DOUBLE_EQ(voltage.shift, 0.0);
    EXPECT_FALSE(voltage.isSigned);
    EXPECT_EQ(voltage.format, SensorFormat::integer);

    // Status registers — without register Name
    ASSERT_EQ(profile.statusRegisters.size(), 2U);
    auto it = profile.statusRegisters.find(500);
    ASSERT_NE(it, profile.statusRegisters.end());
    ASSERT_EQ(it->second.size(), 2U);
    EXPECT_EQ(it->second[0].name, "TestTemp");
    EXPECT_EQ(it->second[0].type, StatusType::sensorFailure);
    EXPECT_EQ(it->second[0].bitPosition, 0U);
    EXPECT_TRUE(it->second[0].value);
    EXPECT_EQ(it->second[1].name, "TestVoltage");
    EXPECT_EQ(it->second[1].type, StatusType::sensorReadingCritical);
    EXPECT_EQ(it->second[1].bitPosition, 3U);
    EXPECT_FALSE(it->second[1].value);

    // Status registers — with register Name prepended to bit names
    auto it2 = profile.statusRegisters.find(501);
    ASSERT_NE(it2, profile.statusRegisters.end());
    ASSERT_EQ(it2->second.size(), 1U);
    EXPECT_EQ(it2->second[0].name, "RPU_PumpFault");
    EXPECT_EQ(it2->second[0].type, StatusType::pumpFailure);
    EXPECT_EQ(it2->second[0].bitPosition, 1U);
    EXPECT_TRUE(it2->second[0].value);

    // Metric registers
    ASSERT_EQ(profile.metricRegisters.size(), 1U);
    const auto& metric = profile.metricRegisters[0];
    EXPECT_EQ(metric.name, "TestClosedDuration");
    EXPECT_EQ(metric.type, MetricType::valveClosedDuration);
    EXPECT_EQ(metric.offset, 700U);
    EXPECT_EQ(metric.size, 2U);
    EXPECT_EQ(metric.precision, 0U);
    EXPECT_DOUBLE_EQ(metric.scale, 60.0);
    EXPECT_DOUBLE_EQ(metric.shift, 0.0);
    EXPECT_TRUE(metric.isSigned);
    EXPECT_EQ(metric.format, SensorFormat::fixedPoint);

    // Firmware registers
    ASSERT_EQ(profile.firmwareRegisters.size(), 1U);
    EXPECT_EQ(profile.firmwareRegisters[0].name, "TestFW");
    EXPECT_EQ(profile.firmwareRegisters[0].type, FirmwareRegisterType::version);
    EXPECT_EQ(profile.firmwareRegisters[0].offset, 600U);
    EXPECT_EQ(profile.firmwareRegisters[0].size, 2U);

    // Config registers — first is periodic, second is one-shot (no Period)
    ASSERT_EQ(profile.configRegisters.size(), 2U);
    const auto& periodic = profile.configRegisters[0];
    EXPECT_EQ(periodic.name, "TestSyncTime");
    EXPECT_EQ(periodic.type, ConfigType::unixTime);
    EXPECT_EQ(periodic.offset, 800U);
    EXPECT_EQ(periodic.size, 2U);
    EXPECT_EQ(periodic.period, std::optional<uint32_t>{3600U});

    // Name is optional; this entry omits it and defaults to "unknown".
    const auto& oneShot = profile.configRegisters[1];
    EXPECT_EQ(oneShot.name, "unknown");
    EXPECT_EQ(oneShot.type, ConfigType::unixTime);
    EXPECT_EQ(oneShot.offset, 801U);
    EXPECT_EQ(oneShot.size, 2U);
    EXPECT_FALSE(oneShot.period.has_value());
}

TEST_F(DeviceProfileTest, CachesProfile)
{
    const auto& p1 = getDeviceProfile(testProfileName);
    const auto& p2 = getDeviceProfile(testProfileName);
    EXPECT_EQ(&p1, &p2);
}

TEST_F(DeviceProfileTest, LoadsRPUProfile)
{
    const auto& profile =
        getDeviceProfile("DeltaRDF040DSS5193E0ReservoirPumpUnit");

    EXPECT_EQ(profile.parity, Parity::none);
    EXPECT_EQ(profile.baudRate, 19200U);
    EXPECT_FALSE(profile.inventoryRegisters.empty());
    EXPECT_FALSE(profile.sensorRegisters.empty());
    EXPECT_FALSE(profile.statusRegisters.empty());
    EXPECT_FALSE(profile.firmwareRegisters.empty());
}

TEST_F(DeviceProfileTest, UnknownTypeThrows)
{
    EXPECT_THROW(getDeviceProfile("NonExistentDevice"), std::runtime_error);
}

TEST_F(DeviceProfileTest, GetDeviceTypeReturnsCorrectType)
{
    EXPECT_EQ(getDeviceType("DeltaRDF040DSS5193E0ReservoirPumpUnit"),
              DeviceType::reservoirPumpUnit);
}

TEST_F(DeviceProfileTest, GetDeviceTypeUnknownThrows)
{
    EXPECT_THROW(getDeviceType("NonExistentDevice"), std::runtime_error);
}

TEST_F(DeviceProfileTest, GetDeviceModelReturnsCorrectModel)
{
    EXPECT_EQ(getDeviceModel("DeltaRDF040DSS5193E0ReservoirPumpUnit"),
              DeviceModel::DeltaRDF040DSS5193E0);
}

TEST_F(DeviceProfileTest, GetDeviceModelUnknownThrows)
{
    EXPECT_THROW(getDeviceModel("NonExistentDevice"), std::runtime_error);
}

TEST_F(DeviceProfileTest, LoadsValveProfile)
{
    const auto& profile = getDeviceProfile("Danfoss003Z8540Valve");

    EXPECT_EQ(profile.parity, Parity::none);
    EXPECT_EQ(profile.baudRate, 19200U);
    EXPECT_FALSE(profile.inventoryRegisters.empty());
    EXPECT_FALSE(profile.sensorRegisters.empty());
    EXPECT_FALSE(profile.statusRegisters.empty());
    EXPECT_FALSE(profile.firmwareRegisters.empty());
    ASSERT_EQ(profile.metricRegisters.size(), 2U);
    EXPECT_EQ(profile.metricRegisters[0].type, MetricType::valveClosedDuration);
    EXPECT_EQ(profile.metricRegisters[1].type, MetricType::valveOpenDuration);

    EXPECT_EQ(getDeviceType("Danfoss003Z8540Valve"), DeviceType::valve);
    EXPECT_EQ(getDeviceModel("Danfoss003Z8540Valve"),
              DeviceModel::Danfoss003Z8540);
}

TEST_F(DeviceProfileTest, LoadsShelfProfile)
{
    const auto& profile = getDeviceProfile("DeltaECD68000049PowerShelf");

    EXPECT_EQ(profile.parity, Parity::even);
    EXPECT_EQ(profile.baudRate, 115200U);
    EXPECT_FALSE(profile.inventoryRegisters.empty());

    EXPECT_EQ(getDeviceType("DeltaECD68000049PowerShelf"),
              DeviceType::powerShelf);
    EXPECT_EQ(getDeviceModel("DeltaECD68000049PowerShelf"),
              DeviceModel::DeltaECD68000049);
}

TEST_F(DeviceProfileTest, GetProfileNamesScansDirectory)
{
    auto names = getProfileNames();
    EXPECT_FALSE(names.empty());
    EXPECT_NE(std::find(names.begin(), names.end(),
                        "DeltaRDF040DSS5193E0ReservoirPumpUnit"),
              names.end());
}
