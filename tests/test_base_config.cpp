#include "base_config.hpp"

#include <sdbusplus/async.hpp>

#include <array>
#include <chrono>
#include <optional>
#include <string>
#include <utility>

#include <gtest/gtest.h>

using namespace std::literals;

namespace ConfigIntf = phosphor::modbus::rtu::config;
namespace EMIntf = entity_manager;

// Mirrors the Entity Manager layout for a second-sourced PSU, where both
// vendor variants sit on the same object, each with its own
// <interface>.RegisterPollRates<N> sub-interfaces.
class BaseConfigTest : public ::testing::Test
{
  protected:
    ~BaseConfigTest() noexcept override = default;

    sdbusplus::async::context ctx;

    static constexpr auto objectPath =
        "/xyz/openbmc_project/inventory/system/board/Ventura2_SCM/PSU_1_1";
    static constexpr auto parentPath =
        "/xyz/openbmc_project/inventory/system/board/Ventura2_SCM";

    static constexpr auto deltaType = "DeltaECD17020037PowerSupplyUnit";
    static constexpr auto artesynType = "Artesyn7000552480000PowerSupplyUnit";

    static auto interfaceFor(const std::string& type) -> std::string
    {
        return "xyz.openbmc_project.Configuration." + type;
    }

    static auto deviceInterface(const std::string& type)
        -> EMIntf::BaseConfigMap
    {
        return {{"Address", uint64_t{144}},
                {"Name", std::string("PSU_1_1")},
                {"SerialPort", std::string("ttyRS485-1")},
                {"Type", type}};
    }

    // One .RegisterPollRates<index> sub-interface entry.
    static auto pollRateInterface(const std::string& type, int index,
                                  const std::string& name, uint64_t rate)
        -> std::pair<std::string, EMIntf::BaseConfigMap>
    {
        return {interfaceFor(type) + ".RegisterPollRates" +
                    std::to_string(index),
                {{"Name", name}, {"PollRate", rate}}};
    }

    // The five PSU status registers, all at the same rate.
    static auto withStatusPollRates(EMIntf::ConfigData& interfaces,
                                    const std::string& type, uint64_t rate)
        -> void
    {
        const std::array names = {"GENERAL_ALARM", "PFC_ALARM", "DCDC_ALARM",
                                  "TEMP_ALARM", "COMM_ALARM"};
        for (size_t i = 0; i < names.size(); i++)
        {
            interfaces.insert(
                pollRateInterface(type, static_cast<int>(i), names[i], rate));
        }
    }

    auto getConfig(const EMIntf::ConfigData& interfaces,
                   const std::string& type)
        -> sdbusplus::async::task<std::optional<ConfigIntf::Config>>
    {
        co_return co_await ConfigIntf::getConfig(
            ctx, sdbusplus::object_path(objectPath), interfaceFor(type),
            interfaces);
    }

    auto testBaseFields(EMIntf::ConfigData interfaces)
        -> sdbusplus::async::task<void>
    {
        auto config = co_await getConfig(interfaces, deltaType);
        if (!config)
        {
            ADD_FAILURE() << "Failed to parse config";
            co_return;
        }
        EXPECT_EQ(config->name, "PSU_1_1");
        EXPECT_EQ(config->type, deltaType);
        EXPECT_EQ(config->address, 0x90);
        EXPECT_EQ(config->serialPort, "ttyRS485-1");
        EXPECT_EQ(config->parentInventoryPath.str, parentPath);
        EXPECT_TRUE(config->registerPollRates.empty());
        co_return;
    }

    auto testDevicePollRate(EMIntf::ConfigData interfaces)
        -> sdbusplus::async::task<void>
    {
        auto config = co_await getConfig(interfaces, deltaType);
        if (!config)
        {
            ADD_FAILURE() << "Failed to parse config";
            co_return;
        }
        EXPECT_EQ(config->pollRate, 3s);
        co_return;
    }

    auto testRegisterPollRates(EMIntf::ConfigData interfaces)
        -> sdbusplus::async::task<void>
    {
        auto config = co_await getConfig(interfaces, deltaType);
        if (!config)
        {
            ADD_FAILURE() << "Failed to parse config";
            co_return;
        }
        EXPECT_EQ(config->registerPollRates.size(), 5U);
        for (const auto& name : {"GENERAL_ALARM", "PFC_ALARM", "DCDC_ALARM",
                                 "TEMP_ALARM", "COMM_ALARM"})
        {
            auto entry = config->registerPollRates.find(name);
            if (entry == config->registerPollRates.end())
            {
                ADD_FAILURE() << "Missing poll rate for " << name;
                continue;
            }
            EXPECT_EQ(entry->second, 45s) << name;
        }
        co_return;
    }

    auto testSecondSource(EMIntf::ConfigData interfaces)
        -> sdbusplus::async::task<void>
    {
        auto delta = co_await getConfig(interfaces, deltaType);
        auto artesyn = co_await getConfig(interfaces, artesynType);
        if (!delta || !artesyn)
        {
            ADD_FAILURE() << "Failed to parse config";
            co_return;
        }
        EXPECT_EQ(delta->registerPollRates.size(), 5U);
        EXPECT_EQ(delta->registerPollRates.at("PFC_ALARM"), 45s);
        EXPECT_EQ(artesyn->registerPollRates.size(), 5U);
        EXPECT_EQ(artesyn->registerPollRates.at("PFC_ALARM"), 30s);
        co_return;
    }

    /** @brief Run a test body and stop the context when it finishes. */
    auto run(sdbusplus::async::task<void> body) -> void
    {
        ctx.spawn(std::move(body) | sdbusplus::async::execution::then([&]() {
                      ctx.request_stop();
                  }));
        ctx.run();
    }
};

// The identity fields come from the device interface and the object path, and
// with no RegisterPollRates sub-interfaces there are no per-register rates.
TEST_F(BaseConfigTest, TestBaseFields)
{
    EMIntf::ConfigData interfaces = {
        {interfaceFor(deltaType), deviceInterface(deltaType)}};
    run(testBaseFields(interfaces));
}

// A PollRate on the device interface is parsed as the device poll rate.
TEST_F(BaseConfigTest, TestDevicePollRate)
{
    auto deviceMap = deviceInterface(deltaType);
    deviceMap.insert({"PollRate", uint64_t{3}});
    EMIntf::ConfigData interfaces = {{interfaceFor(deltaType), deviceMap}};
    run(testDevicePollRate(interfaces));
}

// Every .RegisterPollRates<N> sub-interface is collected, keyed by register
// name.
TEST_F(BaseConfigTest, TestRegisterPollRates)
{
    EMIntf::ConfigData interfaces = {
        {interfaceFor(deltaType), deviceInterface(deltaType)}};
    withStatusPollRates(interfaces, deltaType, 45);
    run(testRegisterPollRates(interfaces));
}

// A second-sourced device carries both vendor interfaces on one object, so
// each config must pick up only the sub-interfaces under its own prefix.
TEST_F(BaseConfigTest, TestSecondSourceInterfacesAreIsolated)
{
    EMIntf::ConfigData interfaces = {
        {interfaceFor(deltaType), deviceInterface(deltaType)},
        {interfaceFor(artesynType), deviceInterface(artesynType)}};
    withStatusPollRates(interfaces, deltaType, 45);
    withStatusPollRates(interfaces, artesynType, 30);
    run(testSecondSource(interfaces));
}
