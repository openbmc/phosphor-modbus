#pragma once

#include "device_profile.hpp"

#include <sdbusplus/async.hpp>

#include <optional>
#include <string>

namespace phosphor::modbus::rtu::config
{

namespace ProfileIntf = phosphor::modbus::rtu::profile;

/** @brief Base configuration for all Modbus RTU devices.
 *  Holds the entity-manager identity fields and a reference to the
 *  device profile.  Both inventory and sensor paths use this struct. */
struct Config
{
    std::string name = "unknown";
    std::string type = "unknown";
    uint8_t address = 0;
    std::string serialPort;
    sdbusplus::object_path inventoryPath;
    const ProfileIntf::DeviceProfile& profile;
};

/** @brief Reads entity-manager properties and looks up the device profile
 *         to build a Config.
 *  @param ctx         The async context.
 *  @param objectPath  The EM object path for this device.
 *  @param interfaceName  The EM interface (type is derived from the last
 *                        segment, e.g.
 * "...DeltaRDF040DSS5193E0ReservoirPumpUnit").
 *  @return A populated Config, or std::nullopt on failure. */
auto getConfig(sdbusplus::async::context& ctx,
               const sdbusplus::object_path& objectPath,
               const std::string& interfaceName)
    -> sdbusplus::async::task<std::optional<Config>>;

} // namespace phosphor::modbus::rtu::config
