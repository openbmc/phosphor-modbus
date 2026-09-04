#pragma once

#include "utils/dump_types.hpp"

#include <nlohmann/json.hpp>

namespace modbus_tool
{

/** @brief Render a dump as the documented schema. See README.md. */
auto toJson(const Dump& dump) -> nlohmann::json;

} // namespace modbus_tool
