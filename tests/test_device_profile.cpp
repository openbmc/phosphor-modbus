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
    "InventoryRegisters": [
        {
            "Name": "TestModel",
            "Type": "Model",
            "Offset": 100,
            "Size": 4
        },
        {
            "Name": "TestSerial",
            "Type": "SerialNumber",
            "Offset": 200,
            "Size": 8
        }
    ],
    "SensorRegisters": [
        {
            "Name": "TestTemp",
            "Type": "Temperature",
            "Offset": 300,
            "Size": 1,
            "Precision": 2,
            "Scale": 0.5,
            "Shift": 10.0,
            "IsSigned": true,
            "Format": "FloatingPoint"
        },
        {
            "Name": "TestVoltage",
            "Type": "Voltage",
            "Offset": 400,
            "Size": 2,
            "Format": "Integer"
        }
    ],
    "StatusRegisters": [
        {
            "Offset": 500,
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
        }
    ],
    "FirmwareRegisters": [
        {
            "Name": "TestFW",
            "Type": "Version",
            "Offset": 600,
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
    EXPECT_EQ(profile.inventoryRegisters[0].name, "TestModel");
    EXPECT_EQ(profile.inventoryRegisters[0].type, InventoryDataType::model);
    EXPECT_EQ(profile.inventoryRegisters[0].offset, 100U);
    EXPECT_EQ(profile.inventoryRegisters[0].size, 4U);
    EXPECT_EQ(profile.inventoryRegisters[1].name, "TestSerial");
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
    EXPECT_EQ(temp.format, SensorFormat::floatingPoint);

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

    // Status registers
    ASSERT_EQ(profile.statusRegisters.size(), 1U);
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

    // Firmware registers
    ASSERT_EQ(profile.firmwareRegisters.size(), 1U);
    EXPECT_EQ(profile.firmwareRegisters[0].name, "TestFW");
    EXPECT_EQ(profile.firmwareRegisters[0].type, FirmwareRegisterType::version);
    EXPECT_EQ(profile.firmwareRegisters[0].offset, 600U);
    EXPECT_EQ(profile.firmwareRegisters[0].size, 2U);
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

TEST_F(DeviceProfileTest, GetProfileNamesScansDirectory)
{
    auto names = getProfileNames();
    EXPECT_FALSE(names.empty());
    EXPECT_NE(std::find(names.begin(), names.end(),
                        "DeltaRDF040DSS5193E0ReservoirPumpUnit"),
              names.end());
}
