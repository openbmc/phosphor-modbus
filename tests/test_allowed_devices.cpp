#include "config/allowed_devices.hpp"

#include <nlohmann/json.hpp>
#include <sdbusplus/async.hpp>

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

using namespace std::literals;
namespace ConfigIntf = phosphor::modbus::rtu::config;

static constexpr auto configDir = "/tmp/phosphor-modbus-test-allowed";
static constexpr auto configFileName = "allowed-devices.json";

class AllowedDevicesTest : public ::testing::Test
{
  protected:
    ~AllowedDevicesTest() noexcept override = default;

    sdbusplus::async::context ctx;

    void SetUp() override
    {
        std::filesystem::create_directories(configDir);
        removeConfig();
    }

    void TearDown() override
    {
        removeConfig();
    }

    void writeConfig(const nlohmann::json& j)
    {
        auto path = std::filesystem::path(configDir) / configFileName;
        std::ofstream file(path);
        file << j.dump();
    }

    void removeConfig()
    {
        auto path = std::filesystem::path(configDir) / configFileName;
        std::filesystem::remove(path);
    }
};

TEST_F(AllowedDevicesTest, NoConfigAllowsAll)
{
    ConfigIntf::AllowedDevices devices(ctx, configDir);

    EXPECT_TRUE(devices.isAllowed("PSU_1_1"));
    EXPECT_TRUE(devices.isAllowed("BBU_SHELF_1"));
    EXPECT_TRUE(devices.isAllowed("anything"));
}

TEST_F(AllowedDevicesTest, AllowlistFiltersDevices)
{
    writeConfig({{"AllowedDevices", {"PSU_1_1", "BBU_SHELF_1"}}});

    ConfigIntf::AllowedDevices devices(ctx, configDir);

    EXPECT_TRUE(devices.isAllowed("PSU_1_1"));
    EXPECT_TRUE(devices.isAllowed("BBU_SHELF_1"));
    EXPECT_FALSE(devices.isAllowed("PSU_1_2"));
    EXPECT_FALSE(devices.isAllowed("CBU_SHELF_1"));
}

TEST_F(AllowedDevicesTest, EmptyAllowlistBlocksAll)
{
    writeConfig({{"AllowedDevices", nlohmann::json::array()}});

    ConfigIntf::AllowedDevices devices(ctx, configDir);

    EXPECT_FALSE(devices.isAllowed("PSU_1_1"));
    EXPECT_FALSE(devices.isAllowed("anything"));
}

TEST_F(AllowedDevicesTest, SpacesReplacedWithUnderscores)
{
    writeConfig({{"AllowedDevices", {"PSU 1 1", "BBU SHELF 1"}}});

    ConfigIntf::AllowedDevices devices(ctx, configDir);

    EXPECT_TRUE(devices.isAllowed("PSU_1_1"));
    EXPECT_TRUE(devices.isAllowed("BBU_SHELF_1"));
}

TEST_F(AllowedDevicesTest, InvalidJsonResetsAllowlist)
{
    auto path = std::filesystem::path(configDir) / configFileName;
    std::ofstream file(path);
    file << "not valid json";
    file.close();

    ConfigIntf::AllowedDevices devices(ctx, configDir);

    EXPECT_TRUE(devices.isAllowed("anything"));
}

TEST_F(AllowedDevicesTest, MissingKeyResetsAllowlist)
{
    writeConfig({{"WrongKey", {"PSU_1_1"}}});

    ConfigIntf::AllowedDevices devices(ctx, configDir);

    EXPECT_TRUE(devices.isAllowed("anything"));
}
