#include "base_config.hpp"

#include "common/entity_manager_interface.hpp"
#include "device_profile.hpp"

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Inventory/Item/client.hpp>

#include <flat_map>

namespace phosphor::modbus::rtu::config
{

PHOSPHOR_LOG2_USING;

using BasicVariantType =
    std::variant<std::vector<std::string>, std::vector<uint8_t>, std::string,
                 int64_t, uint64_t, double, int32_t, uint32_t, int16_t,
                 uint16_t, uint8_t, bool>;
using ConfigMap = std::flat_map<std::string, BasicVariantType>;
using InterfaceData = std::flat_map<std::string, ConfigMap>;
using ManagedObjectType = std::flat_map<sdbusplus::object_path, InterfaceData>;

template <typename T>
static auto getValue(const ConfigMap& configMap, const std::string& key,
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

static auto parseConfig(const ConfigMap& configMap,
                        const sdbusplus::object_path& objectPath,
                        const std::string& interfaceName, std::string type,
                        const ProfileIntf::DeviceProfile& profile)
    -> std::optional<Config>
{
    try
    {
        auto name = getValue<std::string>(configMap, "Name", interfaceName);
        std::replace(name.begin(), name.end(), ' ', '_');

        auto address = getValue<uint64_t>(configMap, "Address", name);

        auto serialPort = getValue<std::string>(configMap, "SerialPort", name);

        return Config{
            .name = std::move(name),
            .type = std::move(type),
            .address = static_cast<uint8_t>(address),
            .serialPort = std::move(serialPort),
            .inventoryPath = objectPath.parent_path(),
            .profile = profile,
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
               const std::string& interfaceName)
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

    using InventoryIntf =
        sdbusplus::client::xyz::openbmc_project::inventory::Item<>;

    constexpr auto entityManager =
        sdbusplus::async::proxy()
            .service(entity_manager::EntityManagerInterface::serviceName)
            .path(InventoryIntf::namespace_path)
            .interface("org.freedesktop.DBus.ObjectManager");

    for (const auto& [path, interfaces] :
         co_await entityManager.call<ManagedObjectType>(ctx,
                                                        "GetManagedObjects"))
    {
        if (path.str != objectPath.str)
        {
            continue;
        }

        auto ifaceIter = interfaces.find(interfaceName);
        if (ifaceIter == interfaces.end())
        {
            error("Interface {INTF} not found at {PATH}", "INTF", interfaceName,
                  "PATH", objectPath);
            co_return std::nullopt;
        }

        co_return parseConfig(ifaceIter->second, objectPath, interfaceName,
                              std::move(type), *profile);
    }

    error("Object path {PATH} not found in EntityManager", "PATH", objectPath);
    co_return std::nullopt;
}

} // namespace phosphor::modbus::rtu::config
