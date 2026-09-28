#pragma once

#include "device_profile.hpp"

#include <xyz/openbmc_project/Metric/Value/aserver.hpp>
#include <xyz/openbmc_project/Sensor/Value/aserver.hpp>

#include <cstdint>
#include <span>
#include <string_view>

namespace phosphor::modbus::rtu::device
{

namespace ProfileIntf = phosphor::modbus::rtu::profile;

using SensorUnit = sdbusplus::common::xyz::openbmc_project::sensor::Value::Unit;
using MetricUnit = sdbusplus::common::xyz::openbmc_project::metric::Value::Unit;

auto getUnitSuffix(ProfileIntf::SensorType type) -> std::string_view;

auto getMetricUnitSuffix(ProfileIntf::MetricType type) -> std::string_view;

/** @brief Returns the sensor unit corresponding to a sensor type.
 *  @throws std::invalid_argument if type is unknown. */
auto getUnit(ProfileIntf::SensorType type) -> SensorUnit;

/** @brief Returns the metric unit corresponding to a metric type.
 *  @throws std::invalid_argument if type is unknown. */
auto getMetricUnit(ProfileIntf::MetricType type) -> MetricUnit;

/** @brief The value of a sensor or metric register. */
auto convertRegisterValue(std::span<const uint16_t> reg,
                          ProfileIntf::SensorFormat format, bool isSigned,
                          uint8_t precision, double scale, double shift)
    -> double;

/** @brief Get the current system time in microseconds since the Epoch.
 *
 *  @return uint64_t equivalent of the system time in microseconds.
 */
auto getCurrentTimeInMicroseconds() -> uint64_t;

} // namespace phosphor::modbus::rtu::device
