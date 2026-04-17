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
