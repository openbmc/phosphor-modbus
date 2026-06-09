#pragma once

#include <sdbusplus/async.hpp>

#include <functional>
#include <memory>
#include <string>

namespace phosphor::modbus
{

class NotifyWatch
{
  public:
    using Callback_t = std::function<sdbusplus::async::task<>(std::string)>;

    NotifyWatch() = delete;
    explicit NotifyWatch(sdbusplus::async::context& ctx, const std::string& dir,
                         Callback_t callback);
    ~NotifyWatch();

    auto readNotifyAsync() -> sdbusplus::async::task<>;

  private:
    sdbusplus::async::context& ctx;
    Callback_t callback;
    int wd = -1;
    int fd = -1;
    std::unique_ptr<sdbusplus::async::fdio> fdioInstance;
};

} // namespace phosphor::modbus
