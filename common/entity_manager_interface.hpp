#pragma once

#include <sdbusplus/async.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <cstdint>
#include <flat_map>
#include <functional>
#include <string>
#include <variant>
#include <vector>

namespace entity_manager
{

using interface_list_t = std::vector<std::string>;

// Entity Manager config data shapes.
using BasicVariantType =
    std::variant<std::vector<std::string>, std::vector<uint8_t>, std::string,
                 int64_t, uint64_t, double, int32_t, uint32_t, int16_t,
                 uint16_t, uint8_t, bool>;
using BaseConfigMap = std::flat_map<std::string, BasicVariantType>;
using ConfigData = std::flat_map<std::string, BaseConfigMap>;
using ManagedObjectType = std::flat_map<sdbusplus::object_path, ConfigData>;

class EntityManagerInterface
{
  public:
    using AddedCallback_t = std::function<sdbusplus::async::task<>(
        const sdbusplus::object_path&, const std::string&, const ConfigData&)>;
    using RemovedCallback_t = std::function<sdbusplus::async::task<>(
        const sdbusplus::object_path&, const std::string&)>;
    static constexpr auto serviceName = "xyz.openbmc_project.EntityManager";

    EntityManagerInterface() = delete;

    explicit EntityManagerInterface(
        sdbusplus::async::context& ctx, const interface_list_t& interfaceNames,
        AddedCallback_t addedCallback, RemovedCallback_t removedCallback);

    /** Get the inventory info from Entity Manager */
    auto handleInventoryGet() -> sdbusplus::async::task<>;

    /** Get the inventory info from Entity Manager for specific interfaces */
    auto handleInventoryGet(const interface_list_t& filterInterfaces)
        -> sdbusplus::async::task<>;

  private:
    /** @brief Handle async inventory add from Entity Manager */
    auto handleInventoryAdded() -> sdbusplus::async::task<>;

    /** @brief Handle async inventory remove from Entity Manager */
    auto handleInventoryRemoved() -> sdbusplus::async::task<>;

    sdbusplus::async::context& ctx;
    interface_list_t interfaceNames;
    AddedCallback_t addedCallback;
    RemovedCallback_t removedCallback;
};

} // namespace entity_manager
