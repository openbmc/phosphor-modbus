#include "base_config.hpp"

#include "common/entity_manager_interface.hpp"
#include "device_profile.hpp"

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Inventory/Item/client.hpp>

#include <flat_map>
#include <unordered_map>

namespace phosphor::modbus::rtu::config
{

PHOSPHOR_LOG2_USING;

using entity_manager::BaseConfigMap;
using entity_manager::ConfigData;
using entity_manager::ManagedObjectType;

template <typename T>
static auto getValue(const BaseConfigMap& configMap, const std::string& key,
                     const std::string& contextName) -> T
{
    auto iter = configMap.find(key);
    if (iter == configMap.end())
    {
        throw std::runtime_error(
            "Missing property " + key + " for " + contextName);
    }

    try
    {
        return std::get<T>(iter->second);
    }
    catch (const std::bad_variant_access&)
    {
        throw std::runtime_error(
            "Incorrect type for property " + key + " in " + contextName);
    }
}

// Get RegisterPollRates from the <interfaceName>.RegisterPollRates<N>
// sub-interfaces.
static auto parseRegisterPollRates(const ConfigData& interfaces,
                                   const std::string& interfaceName)
    -> std::unordered_map<std::string, std::chrono::seconds>
{
    std::unordered_map<std::string, std::chrono::seconds> registerPollRates;
    const auto prefix = interfaceName + ".RegisterPollRates";
    for (const auto& [iface, configMap] : interfaces)
    {
        if (!iface.starts_with(prefix))
        {
            continue;
        }
        try
        {
            auto name = getValue<std::string>(configMap, "Name", iface);
            auto pollRate = std::chrono::seconds(
                getValue<uint64_t>(configMap, "PollRate", iface));
            registerPollRates[name] = pollRate;
        }
        catch (const std::exception& e)
        {
            error("Failed to parse {INTF}: {ERROR}", "INTF", iface, "ERROR", e);
        }
    }
    return registerPollRates;
}

static auto parseConfig(const ConfigData& interfaces,
                        const sdbusplus::object_path& objectPath,
                        const std::string& interfaceName, std::string type,
                        const ProfileIntf::DeviceProfile& profile)
    -> std::optional<Config>
{
    auto ifaceIter = interfaces.find(interfaceName);
    if (ifaceIter == interfaces.end())
    {
        error("Interface {INTF} not found at {PATH}", "INTF", interfaceName,
              "PATH", objectPath);
        return std::nullopt;
    }
    const auto& configMap = ifaceIter->second;

    try
    {
        auto name = std::string(objectPath.filename());

        auto address = getValue<uint64_t>(configMap, "Address", name);

        auto serialPort = getValue<std::string>(configMap, "SerialPort", name);

        auto pollRate = defaultSensorPollInterval;
        auto pollRateIter = configMap.find("PollRate");
        if (pollRateIter != configMap.end())
        {
            pollRate =
                std::chrono::seconds(std::get<uint64_t>(pollRateIter->second));
        }

        return Config{
            .name = std::move(name),
            .type = std::move(type),
            .address = static_cast<uint8_t>(address),
            .serialPort = std::move(serialPort),
            .parentInventoryPath = objectPath.parent_path(),
            .inventoryPath = {},
            .profile = profile,
            .pollRate = pollRate,
            .registerPollRates =
                parseRegisterPollRates(interfaces, interfaceName),
        };
    }
    catch (const std::exception& e)
    {
        error("Failed to parse config for {INTF}: {ERROR}", "INTF",
              interfaceName, "ERROR", e);
        return std::nullopt;
    }
}

auto getConfig(sdbusplus::async::context& ctx,
               const sdbusplus::object_path& objectPath,
               const std::string& interfaceName, const ConfigData& inInterfaces)
    -> sdbusplus::async::task<std::optional<Config>>
{
    auto type = interfaceName.substr(interfaceName.rfind('.') + 1);

    const ProfileIntf::DeviceProfile* profile = nullptr;
    try
    {
        profile = &ProfileIntf::getDeviceProfile(type);
    }
    catch (const std::exception& e)
    {
        error("No device profile for type {TYPE}: {ERROR}", "TYPE", type,
              "ERROR", e);
        co_return std::nullopt;
    }

    // Interface map already provided by the caller (e.g. from the initial
    // GetManagedObjects) - parse it directly without re-querying.
    if (!inInterfaces.empty())
    {
        co_return parseConfig(inInterfaces, objectPath, interfaceName,
                              std::move(type), *profile);
    }

    using InventoryIntf =
        sdbusplus::client::xyz::openbmc_project::inventory::Item<>;

    constexpr auto entityManager =
        sdbusplus::async::proxy()
            .service(entity_manager::EntityManagerInterface::serviceName)
            .path(InventoryIntf::namespace_path)
            .interface("org.freedesktop.DBus.ObjectManager");

    const auto managedObjects = co_await entityManager.call<ManagedObjectType>(
        ctx, "GetManagedObjects");

    for (const auto& [path, interfaces] : managedObjects)
    {
        if (path.str != objectPath.str)
        {
            continue;
        }

        co_return parseConfig(interfaces, objectPath, interfaceName,
                              std::move(type), *profile);
    }

    error("Object path {PATH} not found in EntityManager", "PATH", objectPath);
    co_return std::nullopt;
}

} // namespace phosphor::modbus::rtu::config
