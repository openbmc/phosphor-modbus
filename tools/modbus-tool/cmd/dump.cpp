#include "cmd/dump.hpp"

#include "utils/entity_manager_lookup.hpp"
#include "utils/port_reservation.hpp"
#include "utils/register_reader.hpp"

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace modbus_tool
{

namespace
{

/** @brief A device that failed before any register could be read. */
auto failed(const std::string& name, const std::string& reason) -> DeviceDump
{
    return DeviceDump{
        .name = name, .result = Result::failure, .reason = reason};
}

/** @brief Fill in the identity a failed variant is known by. */
auto failedVariant(const ConfigIntf::Config& config, const std::string& reason)
    -> DeviceDump
{
    return DeviceDump{
        .name = config.name,
        .type = config.type,
        .address = config.address,
        .serialPort = config.serialPort,
        .result = Result::failure,
        .reason = reason,
    };
}

/** @brief Report every variant of every device on a port as failed. */
auto failedPort(const std::vector<const DeviceVariants*>& devices,
                const std::string& reason)
    -> std::map<std::string, std::vector<DeviceDump>>
{
    std::map<std::string, std::vector<DeviceDump>> results;
    for (const auto* device : devices)
    {
        for (const auto& config : device->configs)
        {
            results[device->name].emplace_back(failedVariant(config, reason));
        }
    }
    return results;
}

/** @brief Read every variant of a device and keep the one that answered.
 *
 *  A second sourced device has a configuration per variant and only one is
 *  really present, so the others are dropped once one is identified. If none
 *  match, all of them are reported so the dump shows what was tried. */
auto readDevice(RegisterReader& reader, const DeviceVariants& device)
    -> sdbusplus::async::task<std::vector<DeviceDump>>
{
    std::vector<DeviceDump> attempts;
    for (const auto& config : device.configs)
    {
        auto dump = co_await reader.read(config);
        if (dump.result != Result::failure)
        {
            co_return std::vector<DeviceDump>{std::move(dump)};
        }
        attempts.emplace_back(std::move(dump));
    }
    co_return attempts;
}

/** @brief Read every device on one port, holding the port for all of them.
 *
 *  Reserving a port stops the daemon polling every device on it, so the
 *  reservation is taken once and released when the port is done with. */
auto dumpPort(sdbusplus::async::context& ctx, const std::string& portName,
              const std::vector<const DeviceVariants*>& devices,
              const PortLookup& lookupPortFn)
    -> sdbusplus::async::task<std::map<std::string, std::vector<DeviceDump>>>
{
    static constexpr auto unavailable = "Port unavailable";

    auto port = co_await lookupPortFn(ctx, portName);
    if (!port.config)
    {
        co_return failedPort(devices, unavailable);
    }

    PortReservation reservation(portName);
    if (!co_await reservation.reserve(ctx))
    {
        co_return failedPort(devices, unavailable);
    }

    std::map<std::string, std::vector<DeviceDump>> results;
    RegisterReader reader(ctx, *port.config, port.devicePath);
    if (!reader.ready())
    {
        results = failedPort(devices, unavailable);
    }
    else
    {
        for (const auto* device : devices)
        {
            auto dumps = co_await readDevice(reader, *device);
            auto& into = results[device->name];
            into.insert(into.end(), std::make_move_iterator(dumps.begin()),
                        std::make_move_iterator(dumps.end()));
        }
    }

    co_await reservation.release(ctx);
    co_return results;
}

/** @brief Collect the results in the order the devices were given. */
auto inOrder(const std::vector<DeviceVariants>& devices,
             std::map<std::string, std::vector<DeviceDump>>& results) -> Dump
{
    Dump dump;
    for (const auto& device : devices)
    {
        auto entry = results.find(device.name);
        if (entry == results.end())
        {
            continue;
        }
        dump.devices.insert(dump.devices.end(),
                            std::make_move_iterator(entry->second.begin()),
                            std::make_move_iterator(entry->second.end()));
        results.erase(entry);
    }
    return dump;
}

} // namespace

auto runDump(sdbusplus::async::context& ctx,
             const std::vector<std::string>& names)
    -> sdbusplus::async::task<Dump>
{
    co_return co_await dumpDevices(ctx, co_await lookupDevices(ctx, names),
                                   lookupPort);
}

auto dumpDevices(sdbusplus::async::context& ctx,
                 const std::vector<DeviceVariants>& devices,
                 const PortLookup& lookupPortFn) -> sdbusplus::async::task<Dump>
{
    std::map<std::string, std::vector<const DeviceVariants*>> byPort;
    std::map<std::string, std::vector<DeviceDump>> results;

    for (const auto& device : devices)
    {
        if (device.configs.empty())
        {
            results[device.name].emplace_back(
                failed(device.name, "Not configured"));
            continue;
        }
        // Every variant of a device sits on the same port.
        byPort[device.configs.front().serialPort].emplace_back(&device);
    }

    for (const auto& [portName, onPort] : byPort)
    {
        auto read = co_await dumpPort(ctx, portName, onPort, lookupPortFn);
        for (auto& [name, dumps] : read)
        {
            auto& into = results[name];
            into.insert(into.end(), std::make_move_iterator(dumps.begin()),
                        std::make_move_iterator(dumps.end()));
        }
    }

    co_return inOrder(devices, results);
}

} // namespace modbus_tool
