#pragma once

#include "utils/dump_types.hpp"

#include <nlohmann/json.hpp>

namespace modbus_tool
{

/** @brief Render a dump as the documented schema. See README.md.
 *
 *  Ordered so the keys come out as written rather than sorted, which keeps
 *  what a device is ahead of what it read. */
auto toJson(const Dump& dump) -> nlohmann::ordered_json;

} // namespace modbus_tool
