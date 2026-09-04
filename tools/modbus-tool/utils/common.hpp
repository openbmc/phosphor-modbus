#pragma once

#include "device_profile.hpp"

#include <string_view>

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

/** @brief The name a profile gives an inventory register, which is its type. */
auto inventoryName(ProfileIntf::InventoryDataType type) -> std::string_view;

/** @brief The name a profile gives a status bit's type. */
auto statusTypeName(ProfileIntf::StatusType type) -> std::string_view;

/** @brief The name a profile gives a config register's type. */
auto configTypeName(ProfileIntf::ConfigType type) -> std::string_view;

} // namespace modbus_tool
