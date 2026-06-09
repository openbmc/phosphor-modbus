#pragma once

#include "common/notify_watch.hpp"

#include <sdbusplus/async.hpp>

#include <optional>
#include <string>
#include <unordered_set>

namespace phosphor::modbus::rtu::config
{

class ConnectedDevices
{
  public:
    explicit ConnectedDevices(sdbusplus::async::context& ctx);

    auto isAllowed(const std::string& deviceName) const -> bool;

    auto startWatching() -> void;

  private:
    auto configUpdateHandler(std::string fileName) -> sdbusplus::async::task<>;

    auto loadConfig() -> void;

    sdbusplus::async::context& ctx;
    std::optional<std::unordered_set<std::string>> allowlist;
    NotifyWatch notifyWatch;
};

} // namespace phosphor::modbus::rtu::config
