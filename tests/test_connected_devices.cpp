#include "config/connected_devices.hpp"

#include <nlohmann/json.hpp>
#include <sdbusplus/async.hpp>

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

using namespace std::literals;
namespace ConfigIntf = phosphor::modbus::rtu::config;

static constexpr auto configDir = CONFIG_DIR;
static constexpr auto configFileName = "connected-devices.json";

class ConnectedDevicesTest : public ::testing::Test
{
  protected:
    ~ConnectedDevicesTest() noexcept override = default;

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

TEST_F(ConnectedDevicesTest, NoConfigAllowsAll)
{
    ConfigIntf::ConnectedDevices devices(ctx);

    EXPECT_TRUE(devices.isAllowed("PSU_1_1"));
    EXPECT_TRUE(devices.isAllowed("BBU_SHELF_1"));
    EXPECT_TRUE(devices.isAllowed("anything"));
}

TEST_F(ConnectedDevicesTest, AllowlistFiltersDevices)
{
    writeConfig({{"ConnectedDevices", {"PSU_1_1", "BBU_SHELF_1"}}});

    ConfigIntf::ConnectedDevices devices(ctx);

    EXPECT_TRUE(devices.isAllowed("PSU_1_1"));
    EXPECT_TRUE(devices.isAllowed("BBU_SHELF_1"));
    EXPECT_FALSE(devices.isAllowed("PSU_1_2"));
    EXPECT_FALSE(devices.isAllowed("CBU_SHELF_1"));
}

TEST_F(ConnectedDevicesTest, EmptyAllowlistBlocksAll)
{
    writeConfig({{"ConnectedDevices", nlohmann::json::array()}});

    ConfigIntf::ConnectedDevices devices(ctx);

    EXPECT_FALSE(devices.isAllowed("PSU_1_1"));
    EXPECT_FALSE(devices.isAllowed("anything"));
}

TEST_F(ConnectedDevicesTest, SpacesReplacedWithUnderscores)
{
    writeConfig({{"ConnectedDevices", {"PSU 1 1", "BBU SHELF 1"}}});

    ConfigIntf::ConnectedDevices devices(ctx);

    EXPECT_TRUE(devices.isAllowed("PSU_1_1"));
    EXPECT_TRUE(devices.isAllowed("BBU_SHELF_1"));
}

TEST_F(ConnectedDevicesTest, InvalidJsonResetsAllowlist)
{
    auto path = std::filesystem::path(configDir) / configFileName;
    std::ofstream file(path);
    file << "not valid json";
    file.close();

    ConfigIntf::ConnectedDevices devices(ctx);

    EXPECT_TRUE(devices.isAllowed("anything"));
}

TEST_F(ConnectedDevicesTest, MissingKeyResetsAllowlist)
{
    writeConfig({{"WrongKey", {"PSU_1_1"}}});

    ConfigIntf::ConnectedDevices devices(ctx);

    EXPECT_TRUE(devices.isAllowed("anything"));
}
