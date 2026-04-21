#include "device_profile.hpp"

#include "profiles/delta_RDF040DSS5193E0_rpu.hpp"

#include <stdexcept>
#include <unordered_map>

namespace phosphor::modbus::rtu::profile
{

struct DeviceProfileEntry
{
    DeviceType deviceType;
    DeviceModel deviceModel;
    const DeviceProfile& profile;
};

static const std::unordered_map<std::string_view, DeviceProfileEntry>
    deviceProfiles = {
        {"DeltaRDF040DSS5193E0ReservoirPumpUnit",
         {DeviceType::reservoirPumpUnit, DeviceModel::DeltaRDF040DSS5193E0,
          deltaRdf040dss5193e0Rpu}},
};

static auto findEntry(std::string_view type) -> const DeviceProfileEntry&
{
    auto it = deviceProfiles.find(type);
    if (it == deviceProfiles.end())
    {
        throw std::out_of_range("Unknown device type: " + std::string(type));
    }
    return it->second;
}

auto getDeviceProfile(std::string_view type) -> const DeviceProfile&
{
    return findEntry(type).profile;
}

auto getDeviceType(std::string_view type) -> DeviceType
{
    return findEntry(type).deviceType;
}

auto getDeviceModel(std::string_view type) -> DeviceModel
{
    return findEntry(type).deviceModel;
}

auto getProfileNames() -> std::vector<std::string_view>
{
    std::vector<std::string_view> names;
    names.reserve(deviceProfiles.size());
    for (const auto& [key, _] : deviceProfiles)
    {
        names.emplace_back(key);
    }
    return names;
}

} // namespace phosphor::modbus::rtu::profile
