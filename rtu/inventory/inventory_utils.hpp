#pragma once

#include "device_profile.hpp"

#include <cstdint>
#include <span>

namespace phosphor::modbus::rtu::inventory
{

namespace ProfileIntf = phosphor::modbus::rtu::profile;

/** @brief Whether a probe register read holds the value the profile expects,
 *         identifying the device as that type. */
auto matchesProbeValue(std::span<const uint16_t> readBuffer,
                       const ProfileIntf::ProbeRegister& probe) -> bool;

} // namespace phosphor::modbus::rtu::inventory
