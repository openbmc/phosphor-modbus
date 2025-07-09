#include "entity_manager_interface.hpp"

#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <xyz/openbmc_project/Inventory/Item/client.hpp>

#include <algorithm>
#include <utility>

namespace entity_manager
{

PHOSPHOR_LOG2_USING;

namespace rules_intf = sdbusplus::bus::match::rules;

using BasicVariantType =
    std::variant<std::vector<std::string>, std::vector<uint8_t>, std::string,
                 int64_t, uint64_t, double, int32_t, uint32_t, int16_t,
                 uint16_t, uint8_t, bool>;
using BaseConfigMap = boost::container::flat_map<std::string, BasicVariantType>;
using ConfigData = boost::container::flat_map<std::string, BaseConfigMap>;
using ManagedObjectType =
    boost::container::flat_map<sdbusplus::message::object_path, ConfigData>;

EntityManagerInterface::EntityManagerInterface(
    sdbusplus::async::context& ctx, const interface_list_t& interfaceNames,
    Callback_t addedCallback, Callback_t removedCallback) :
    ctx(ctx), interfaceNames(interfaceNames),
    addedCallback(std::move(addedCallback)),
    removedCallback(std::move(removedCallback))
{
    ctx.spawn(handleInventoryAdded());
    ctx.spawn(handleInventoryRemoved());
}

auto EntityManagerInterface::handleInventoryGet() -> sdbusplus::async::task<>
{
    if (!addedCallback)
    {
        error("addedCallback is not set");
        co_return;
    }

    using InventoryIntf =
        sdbusplus::client::xyz::openbmc_project::inventory::Item<>;

    constexpr auto entityManager =
        sdbusplus::async::proxy()
            .service(serviceName)
            .path(InventoryIntf::namespace_path)
            .interface("org.freedesktop.DBus.ObjectManager");

    for (const auto& [objectPath, interfaceConfig] :
         co_await entityManager.call<ManagedObjectType>(ctx,
                                                        "GetManagedObjects"))
    {
        for (const auto& interfaceName : interfaceNames)
        {
            if (interfaceConfig.contains(interfaceName))
            {
                co_await addedCallback(objectPath, interfaceName);
            }
        }
    }

    co_return;
}

auto EntityManagerInterface::handleInventoryAdded() -> sdbusplus::async::task<>
{
    if (!addedCallback)
    {
        error("addedCallback is not set");
        co_return;
    }

    auto addedMatch = sdbusplus::async::match(
        ctx, rules_intf::interfacesAdded() + rules_intf::sender(serviceName));

    while (!ctx.stop_requested())
    {
        auto result = co_await addedMatch
                          .next<sdbusplus::message::object_path, ConfigData>();
        auto& [objectPath, inventoryData] = result;

        for (const auto& interfaceName : interfaceNames)
        {
            if (inventoryData.contains(interfaceName))
            {
                co_await addedCallback(objectPath, interfaceName);
            }
        }
    }

    co_return;
}

auto EntityManagerInterface::handleInventoryRemoved()
    -> sdbusplus::async::task<>
{
    if (!removedCallback)
    {
        error("removedCallback is not set");
        co_return;
    }

    auto removedMatch = sdbusplus::async::match(
        ctx, rules_intf::interfacesRemoved() + rules_intf::sender(serviceName));

    while (!ctx.stop_requested())
    {
        auto result =
            co_await removedMatch
                .next<sdbusplus::message::object_path, interface_list_t>();
        auto& [objectPath, interfaces] = result;

        for (const auto& interfaceName : interfaceNames)
        {
            if (std::ranges::find(interfaces, interfaceName) !=
                interfaces.end())
            {
                co_await removedCallback(objectPath, interfaceName);
            }
        }
    }

    co_return;
}

} // namespace entity_manager
