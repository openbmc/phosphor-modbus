#pragma once

#include "common/notify_watch.hpp"

#include <sdbusplus/async.hpp>

#include <optional>
#include <string>
#include <unordered_set>

namespace phosphor::modbus::rtu::config
{

class AllowedDevices
{
  public:
    AllowedDevices(sdbusplus::async::context& ctx,
                   const std::string& configDir);

    auto isAllowed(const std::string& deviceName) const -> bool;

    /** @brief The device names read from the allowlist config.
     *  - nullopt: no usable config present, either missing or unparseable,
     *    so every device is allowed.
     *  - empty set: config present but lists no devices, so no device is
     *    allowed.
     *  - non-empty set: only the listed devices are allowed.
     *
     *  NOTE: nullopt and an empty set are different in their meaning, so
     *  use isAllowed() to explicitly understand whether a device is
     *  allowed. */
    auto getConfiguredDevices() const
        -> const std::optional<std::unordered_set<std::string>>&;

    auto startWatching() -> void;

  private:
    auto configUpdateHandler(std::string fileName) -> sdbusplus::async::task<>;

    auto loadConfig() -> void;

    sdbusplus::async::context& ctx;
    std::string configDir;
    std::optional<std::unordered_set<std::string>> allowlist;
    NotifyWatch notifyWatch;
};

} // namespace phosphor::modbus::rtu::config
