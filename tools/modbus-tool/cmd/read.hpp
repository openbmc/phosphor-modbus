#pragma once

#include "utils/entity_manager_lookup.hpp"

#include <nlohmann/json.hpp>
#include <sdbusplus/async.hpp>

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace modbus_tool
{

/** @brief How a read ended: success, or the stage at which it stopped. */
enum class ReadStatus
{
    success,
    notConfigured,
    portUnavailable,
    noResponse,
    probeMismatch,
    failed,
};

/** @brief The registers requested and the outcome of reading them. */
struct ReadResult
{
    ReadStatus status = ReadStatus::portUnavailable;
    /** @brief Offset of the first register requested. */
    uint16_t offset = 0;
    /** @brief Number of registers requested. */
    uint16_t size = 0;
    /** @brief The values read, one per register starting at offset. Filled
     *  only on success. */
    std::vector<uint16_t> registers{};
};

/** @brief The text a status is reported as, on stderr and as Reason in the
 *  JSON. */
auto readStatusMessage(ReadStatus status) -> std::string_view;

/** @brief Render a read as the object schemas/read.json describes. */
auto toJson(const ReadResult& result) -> nlohmann::ordered_json;

/** @brief Look the named device up in entity-manager and read size
 *  registers from offset, as readDevice does. */
auto runRead(sdbusplus::async::context& ctx, const std::string& name,
             uint16_t offset, uint16_t size)
    -> sdbusplus::async::task<ReadResult>;

/** @brief Read size registers from offset on a device.
 *
 *  Resolves the device's port with lookupPortFn and reserves it, probes each
 *  variant to find the one present, reads from that variant in a single
 *  request, then releases the port. */
auto readDevice(sdbusplus::async::context& ctx, const DeviceVariants& device,
                uint16_t offset, uint16_t size, const PortLookup& lookupPortFn)
    -> sdbusplus::async::task<ReadResult>;

} // namespace modbus_tool
