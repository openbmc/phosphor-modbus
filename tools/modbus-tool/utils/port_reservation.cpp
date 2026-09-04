#include "utils/port_reservation.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/bus.hpp>
#include <xyz/openbmc_project/Inventory/Item/client.hpp>
#include <xyz/openbmc_project/Object/Enable/client.hpp>

#include <algorithm>
#include <chrono>
#include <utility>
#include <variant>

namespace modbus_tool
{

PHOSPHOR_LOG2_USING;

using namespace std::literals;
using EnableIntf = sdbusplus::client::xyz::openbmc_project::object::Enable<>;
using InventoryIntf =
    sdbusplus::client::xyz::openbmc_project::inventory::Item<>;

namespace
{
constexpr auto propertyName = "Enabled";
// The wait covers operations already on the wire, including their retries.
constexpr auto pollInterval = 100ms;
constexpr int pollRetries = 100;

auto connectorPath(std::string name) -> std::string
{
    // D-Bus paths can't contain '-' (e.g. ttyRS485-8 -> ttyRS485_8).
    std::ranges::replace(name, '-', '_');
    return (sdbusplus::object_path(InventoryIntf::namespace_path) / "system" /
            "connector" / name)
        .str;
}
} // namespace

PortReservation::PortReservation(std::string portName) :
    portName(std::move(portName)), objectPath(connectorPath(this->portName))
{}

PortReservation::~PortReservation()
{
    // Nothing to do unless release() was missed, which means the run is being
    // torn down and the context can no longer carry the call.
    releaseBlocking();
}

auto PortReservation::reserve(sdbusplus::async::context& ctx)
    -> sdbusplus::async::task<bool>
{
    auto port = sdbusplus::async::proxy()
                    .service(daemonService)
                    .path(objectPath)
                    .interface(EnableIntf::interface);

    co_await port.set_property(ctx, propertyName, false);

    // The write only reserves the port. It reads back false once the bus has
    // gone quiet, and stays true if another client holds it.
    for (int i = 0; i < pollRetries; i++)
    {
        if (!co_await port.get_property<bool>(ctx, propertyName))
        {
            held = true;
            co_return true;
        }
        co_await sdbusplus::async::sleep_for(ctx, pollInterval);
    }

    co_return false;
}

auto PortReservation::release(sdbusplus::async::context& ctx)
    -> sdbusplus::async::task<void>
{
    if (!held)
    {
        co_return;
    }
    held = false;

    co_await sdbusplus::async::proxy()
        .service(daemonService)
        .path(objectPath)
        .interface(EnableIntf::interface)
        .set_property(ctx, propertyName, true);
}

auto PortReservation::releaseBlocking() -> void
{
    if (!held)
    {
        return;
    }
    held = false;

    // A connection of its own, so this still works once the context has
    // stopped.
    try
    {
        auto bus = sdbusplus::bus::new_default();
        auto method =
            bus.new_method_call(daemonService, objectPath.c_str(),
                                "org.freedesktop.DBus.Properties", "Set");
        method.append(EnableIntf::interface, propertyName,
                      std::variant<bool>(true));
        bus.call(method);
    }
    catch (const std::exception& e)
    {
        error("Failed to release port {PORT}: {ERROR}", "PORT", portName,
              "ERROR", e);
    }
}

} // namespace modbus_tool
