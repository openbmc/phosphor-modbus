#pragma once

#include "device_profile.hpp"

#include <string_view>

namespace phosphor::modbus::rtu::device
{

namespace ProfileIntf = phosphor::modbus::rtu::profile;

auto getUnitSuffix(ProfileIntf::SensorType type) -> std::string_view;

auto getMetricUnitSuffix(ProfileIntf::MetricType type) -> std::string_view;

} // namespace phosphor::modbus::rtu::device
