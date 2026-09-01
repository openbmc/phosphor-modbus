#pragma once

#include "device_profile.hpp"

#include <cstdint>
#include <span>
#include <string_view>

namespace phosphor::modbus::rtu::device
{

namespace ProfileIntf = phosphor::modbus::rtu::profile;

auto getUnitSuffix(ProfileIntf::SensorType type) -> std::string_view;

auto getMetricUnitSuffix(ProfileIntf::MetricType type) -> std::string_view;

/** @brief Whether a probe register read holds the value the profile expects,
 *         identifying the device as that type. */
auto matchesProbeValue(std::span<const uint16_t> readBuffer,
                       const ProfileIntf::ProbeRegister& probe) -> bool;

} // namespace phosphor::modbus::rtu::device
