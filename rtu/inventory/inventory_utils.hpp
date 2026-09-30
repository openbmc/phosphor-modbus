#pragma once

#include "device_profile.hpp"

#include <cstdint>
#include <span>
#include <string>

namespace phosphor::modbus::rtu::inventory
{

namespace ProfileIntf = phosphor::modbus::rtu::profile;

/** @brief Whether a probe register read holds the value the profile expects,
 *         identifying the device as that type. */
auto matchesProbeValue(std::span<const uint16_t> readBuffer,
                       const ProfileIntf::ProbeRegister& probe) -> bool;

/** @brief The string an inventory register holds. */
auto convertRegisterValue(std::span<const uint16_t> registers,
                          const ProfileIntf::InventoryRegister& reg)
    -> std::string;

} // namespace phosphor::modbus::rtu::inventory
