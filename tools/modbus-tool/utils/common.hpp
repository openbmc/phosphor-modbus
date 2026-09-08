#pragma once

#include "device_profile.hpp"

#include <sdbusplus/async.hpp>

#include <expected>
#include <string>
#include <string_view>
#include <vector>

namespace modbus_tool
{

namespace ProfileIntf = phosphor::modbus::rtu::profile;

/** @brief Allows one invocation of the tool at a time.
 *
 *  A port reservation cannot tell two invocations apart, so one would release
 *  a reservation the other owns. Held for the whole run, and by the kernel, so
 *  it cannot go stale. */
class InstanceLock
{
  public:
    InstanceLock() = default;
    InstanceLock(const InstanceLock&) = delete;
    InstanceLock& operator=(const InstanceLock&) = delete;
    InstanceLock(InstanceLock&&) = delete;
    InstanceLock& operator=(InstanceLock&&) = delete;
    ~InstanceLock();

    /** @brief Take the lock.
     *  @return False if another instance holds it, or it is unavailable. */
    auto acquire() -> bool;

  private:
    int fd = -1;
};

/** @brief The devices the platform's allowlist names, sorted.
 *
 *  This is what the daemon is permitted to poll, so it is what the tool reads
 *  when asked for every device.
 *  @return The names, or why there are none to read. */
auto allowedDeviceNames(sdbusplus::async::context& ctx,
                        const std::string& configDir)
    -> std::expected<std::vector<std::string>, std::string>;

/** @brief The name a profile gives an inventory register, which is its type. */
auto inventoryName(ProfileIntf::InventoryDataType type) -> std::string_view;

/** @brief The name a profile gives a status bit's type. */
auto statusTypeName(ProfileIntf::StatusType type) -> std::string_view;

/** @brief The name a profile gives a config register's type. */
auto configTypeName(ProfileIntf::ConfigType type) -> std::string_view;

} // namespace modbus_tool
