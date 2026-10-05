#pragma once

#include "utils/entity_manager_lookup.hpp"

#include <sdbusplus/async.hpp>

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace modbus_tool
{

/** @brief How a write ended.
 *
 *  Only success says the device acknowledged the write. The device is probed
 *  first, so the stage that failed says what went wrong without the write
 *  having to guess. */
enum class WriteStatus
{
    success,
    notConfigured,
    portUnavailable,
    noResponse,
    probeMismatch,
    rejected,
};

/** @brief What a status reads as on stderr. */
auto writeStatusMessage(WriteStatus status) -> std::string_view;

/** @brief Write registers to one device, reserving its port first.
 *
 *  The device is probed before anything is written, so a name that resolves
 *  to an absent device, or to a variant that is not the one present, is
 *  reported rather than written to. Nothing is read back: the result says the
 *  device acknowledged the write, not that the register holds the value. */
auto runWrite(sdbusplus::async::context& ctx, const std::string& name,
              uint16_t offset, std::span<const uint16_t> values)
    -> sdbusplus::async::task<WriteStatus>;

/** @brief The write itself, given the device and a way to reach its port.
 *  Kept apart from runWrite so the flow does not have to discover its own
 *  inputs. */
auto writeDevice(sdbusplus::async::context& ctx, const DeviceVariants& device,
                 uint16_t offset, std::span<const uint16_t> values,
                 const PortLookup& lookupPortFn)
    -> sdbusplus::async::task<WriteStatus>;

} // namespace modbus_tool
