#include "device_profile.hpp"

#include "profiles/delta_RDF040DSS5193E0_rpu.hpp"

#include <stdexcept>
#include <unordered_map>

namespace phosphor::modbus::rtu::profile
{

static const std::unordered_map<std::string_view, const DeviceProfile&>
    deviceProfiles = {
        {"DeltaRDF040DSS5193E0ReservoirPumpUnit", deltaRdf040dss5193e0Rpu},
};

auto getDeviceProfile(std::string_view type) -> const DeviceProfile&
{
    auto it = deviceProfiles.find(type);
    if (it == deviceProfiles.end())
    {
        throw std::out_of_range("Unknown device type: " + std::string(type));
    }
    return it->second;
}

} // namespace phosphor::modbus::rtu::profile
