#pragma once

#include "device_profile.hpp"

#include <cstdint>
#include <span>
#include <string>

namespace phosphor::modbus::rtu::device
{

namespace ProfileIntf = phosphor::modbus::rtu::profile;

/** @brief The version a firmware register holds. */
auto convertRegisterValue(std::span<const uint16_t> registers,
                          const ProfileIntf::FirmwareRegister& reg)
    -> std::string;

} // namespace phosphor::modbus::rtu::device
