#include "device_profile.hpp"
#include "profiles/delta_RDF040DSS5193E0_rpu.hpp"

#include <gtest/gtest.h>

using namespace phosphor::modbus::rtu::profile;

TEST(DeviceProfileTest, KnownTypeReturnsProfile)
{
    const auto& profile =
        getDeviceProfile("DeltaRDF040DSS5193E0ReservoirPumpUnit");
    const auto& expected = deltaRdf040dss5193e0Rpu;

    EXPECT_EQ(&profile, &expected)
        << "Lookup did not return the expected profile instance";
}

TEST(DeviceProfileTest, UnknownTypeThrows)
{
    EXPECT_THROW(getDeviceProfile("NonExistentDevice"), std::out_of_range);
}

TEST(DeviceProfileTest, GetDeviceTypeReturnsCorrectType)
{
    EXPECT_EQ(getDeviceType("DeltaRDF040DSS5193E0ReservoirPumpUnit"),
              DeviceType::reservoirPumpUnit);
}

TEST(DeviceProfileTest, GetDeviceTypeUnknownThrows)
{
    EXPECT_THROW(getDeviceType("NonExistentDevice"), std::out_of_range);
}

TEST(DeviceProfileTest, GetDeviceModelReturnsCorrectModel)
{
    EXPECT_EQ(getDeviceModel("DeltaRDF040DSS5193E0ReservoirPumpUnit"),
              DeviceModel::DeltaRDF040DSS5193E0);
}

TEST(DeviceProfileTest, GetDeviceModelUnknownThrows)
{
    EXPECT_THROW(getDeviceModel("NonExistentDevice"), std::out_of_range);
}

TEST(DeviceProfileTest, GetProfileNamesReturnsAllProfiles)
{
    auto names = getProfileNames();
    EXPECT_FALSE(names.empty());
    EXPECT_NE(std::find(names.begin(), names.end(),
                        "DeltaRDF040DSS5193E0ReservoirPumpUnit"),
              names.end());
}
