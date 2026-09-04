#include "utils/entity_manager_lookup.hpp"

#include "common/entity_manager_interface.hpp"
#include "device_profile.hpp"

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Inventory/Item/client.hpp>

#include <utility>

namespace modbus_tool
{

PHOSPHOR_LOG2_USING;

namespace ProfileIntf = phosphor::modbus::rtu::profile;

using entity_manager::ManagedObjectType;
using InventoryIntf =
    sdbusplus::client::xyz::openbmc_project::inventory::Item<>;

namespace
{

constexpr auto configPrefix = "xyz.openbmc_project.Configuration.";

/** @brief The entity-manager interface for every installed profile, which is
 *  the same set the daemon watches. */
auto deviceInterfaces() -> std::vector<std::string>
{
    std::vector<std::string> interfaces;
    for (const auto& name : ProfileIntf::getProfileNames())
    {
        interfaces.emplace_back(configPrefix + name);
    }
    return interfaces;
}

auto managedObjects(sdbusplus::async::context& ctx)
    -> sdbusplus::async::task<ManagedObjectType>
{
    constexpr auto entityManager =
        sdbusplus::async::proxy()
            .service(entity_manager::EntityManagerInterface::serviceName)
            .path(InventoryIntf::namespace_path)
            .interface("org.freedesktop.DBus.ObjectManager");

    co_return co_await entityManager.call<ManagedObjectType>(
        ctx, "GetManagedObjects");
}

} // namespace

auto lookupDevices(sdbusplus::async::context& ctx,
                   const std::vector<std::string>& names)
    -> sdbusplus::async::task<std::vector<DeviceVariants>>
{
    const auto objects = co_await managedObjects(ctx);
    const auto interfaces = deviceInterfaces();

    std::vector<DeviceVariants> found;
    found.reserve(names.size());

    for (const auto& name : names)
    {
        DeviceVariants variants;
        variants.name = name;

        for (const auto& [objectPath, objectInterfaces] : objects)
        {
            if (objectPath.filename() != name)
            {
                continue;
            }

            for (const auto& interfaceName : interfaces)
            {
                if (!objectInterfaces.contains(interfaceName))
                {
                    continue;
                }

                auto config = co_await ConfigIntf::getConfig(
                    ctx, objectPath, interfaceName, objectInterfaces);
                if (config)
                {
                    variants.configs.emplace_back(std::move(*config));
                }
            }
        }

        found.emplace_back(std::move(variants));
    }

    co_return found;
}

auto lookupPort(sdbusplus::async::context& ctx, const std::string& portName)
    -> sdbusplus::async::task<PortDetails>
{
    const auto objects = co_await managedObjects(ctx);
    const auto interfaces = PortIntf::PortFactory::getInterfaces();

    for (const auto& [objectPath, objectInterfaces] : objects)
    {
        for (const auto& interfaceName : interfaces)
        {
            if (!objectInterfaces.contains(interfaceName))
            {
                continue;
            }

            auto config = co_await PortIntf::PortFactory::getConfig(
                ctx, objectPath, interfaceName);
            if (!config || config->name != portName)
            {
                continue;
            }

            PortDetails port;
            try
            {
                port.devicePath = PortIntf::PortFactory::getDevicePath(*config);
            }
            catch (const std::exception& e)
            {
                error("No device for port {PORT}: {ERROR}", "PORT", portName,
                      "ERROR", e);
                co_return PortDetails{};
            }

            port.config = std::move(config);
            co_return port;
        }
    }

    co_return PortDetails{};
}

} // namespace modbus_tool
