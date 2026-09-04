#pragma once

#include "utils/dump_types.hpp"
#include "utils/entity_manager_lookup.hpp"

#include <sdbusplus/async.hpp>

#include <functional>
#include <string>
#include <vector>

namespace modbus_tool
{

/** @brief Dump the named devices, reserving each port once.
 *
 *  Devices are reported in the order they were named. A device that cannot be
 *  read is reported as a failure rather than abandoning the run, so the result
 *  is empty only when nothing could be attempted at all. */
auto runDump(sdbusplus::async::context& ctx,
             const std::vector<std::string>& names)
    -> sdbusplus::async::task<Dump>;

/** @brief How a port name is turned into something readable. */
using PortLookup = std::function<sdbusplus::async::task<PortDetails>(
    sdbusplus::async::context&, const std::string&)>;

/** @brief The dump itself, given the devices to read and a way to reach their
 *  ports. Kept apart from runDump so the flow does not have to discover its
 *  own inputs. */
auto dumpDevices(sdbusplus::async::context& ctx,
                 const std::vector<DeviceVariants>& devices,
                 const PortLookup& lookupPortFn)
    -> sdbusplus::async::task<Dump>;

} // namespace modbus_tool
