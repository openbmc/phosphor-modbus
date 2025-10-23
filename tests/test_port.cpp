#include "common/entity_manager_interface.hpp"
#include "modbus_server_tester.hpp"
#include "port/port_factory.hpp"

#include <fcntl.h>

#include <xyz/openbmc_project/Configuration/USBPort/aserver.hpp>
#include <xyz/openbmc_project/Inventory/Item/client.hpp>

#include <gtest/gtest.h>

using namespace std::literals;

namespace phosphor::modbus::rtu::port::config
{

// Local copy of the USBPortConfig struct definition
struct TestPortConfig : public PortFactoryConfig
{
    std::string address = "unknown";
    uint16_t port = 0;
    uint16_t interface = 0;

    virtual ~TestPortConfig() = default;
};

} // namespace phosphor::modbus::rtu::port::config

class PortTest;

namespace TestIntf = phosphor::modbus::test;
namespace PortIntf = phosphor::modbus::rtu::port;
namespace PortConfigIntf = PortIntf::config;
namespace RTUIntf = phosphor::modbus::rtu;
using PortFactoryIntf = PortIntf::PortFactory;
using USBPortConfigServerIntf =
    sdbusplus::aserver::xyz::openbmc_project::configuration::USBPort<PortTest>;

struct properties_t
{
    std::string name = {};
    std::string mode = {};
    uint64_t baud_rate = {};
    uint64_t rts_delay = {};
};

class MockPort : public PortIntf::BasePort
{
  public:
    MockPort(sdbusplus::async::context& ctx,
             const PortConfigIntf::Config& config,
             const std::string& devicePath) : BasePort(ctx, config, devicePath)
    {}
};

class PortTest : public ::testing::Test
{
  public:
    static constexpr properties_t properties = {"TestPort", "RS485", 115200, 1};
    static constexpr const char* clientDevicePath = "/tmp/ttyPortV0";
    static constexpr const char* serverDevicePath = "/tmp/ttyPortV1";
    static constexpr const auto defaultBaudeRate = "b115200";
    int socat_pid = -1;
    sdbusplus::async::context ctx;
    int fdClient = -1;
    std::unique_ptr<TestIntf::ServerTester> serverTester;
    int fdServer = -1;
    bool getPortConfigPassed = false;

    PortTest()
    {
        std::string socatCmd = std::format(
            "socat -x -v -d -d pty,link={},rawer,echo=0,parenb,{} pty,link={},rawer,echo=0,parenb,{} & echo $!",
            serverDevicePath, defaultBaudeRate, clientDevicePath,
            defaultBaudeRate);

        // Start socat in the background and capture its PID
        FILE* fp = popen(socatCmd.c_str(), "r");
        EXPECT_NE(fp, nullptr) << "Failed to start socat: " << strerror(errno);
        EXPECT_GT(fscanf(fp, "%d", &socat_pid), 0);
        pclose(fp);

        // Wait for socat to start up
        sleep(1);

        fdClient = open(clientDevicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
        EXPECT_NE(fdClient, -1)
            << "Failed to open serial port " << clientDevicePath
            << " with error: " << strerror(errno);

        fdServer = open(serverDevicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
        EXPECT_NE(fdServer, -1)
            << "Failed to open serial port " << serverDevicePath
            << " with error: " << strerror(errno);

        serverTester = std::make_unique<TestIntf::ServerTester>(ctx, fdServer);
    }

    ~PortTest() noexcept override
    {
        if (fdClient != -1)
        {
            close(fdClient);
            fdClient = -1;
        }
        if (fdServer != -1)
        {
            close(fdServer);
            fdServer = -1;
        }
        kill(socat_pid, SIGTERM);
    }

    void SetUp() override
    {
        getPortConfigPassed = false;
    }

    auto TestHoldingRegisters(PortConfigIntf::Config& config, MockPort& port,
                              uint16_t registerOffset, bool res)
        -> sdbusplus::async::task<void>
    {
        std::vector<uint16_t> registers(
            TestIntf::testSuccessReadHoldingRegisterCount);

        auto ret = co_await port.readHoldingRegisters(
            TestIntf::testDeviceAddress, registerOffset, config.baudRate,
            RTUIntf::Parity::none, registers);

        EXPECT_EQ(ret, res) << "Failed to read holding registers";

        if (!res)
        {
            co_return;
        }

        for (auto i = 0; i < TestIntf::testSuccessReadHoldingRegisterCount; i++)
        {
            EXPECT_EQ(registers[i],
                      TestIntf::testSuccessReadHoldingRegisterResponse[i]);
        }

        co_return;
    }

    template <typename Config, typename Properties>
    static inline void VerifyConfig(const Config& config,
                                    const Properties& property)
    {
        EXPECT_EQ(config, property);
    }

    auto TestGetUSBPortConfig(
        const USBPortConfigServerIntf::properties_t properties, bool shouldPass)
        -> sdbusplus::async::task<void>
    {
        static constexpr auto objectPath =
            "/xyz/openbmc_project/inventory/system/board/Ventura_Modbus/DevTTYUSB0";

        auto configServer = std::make_unique<USBPortConfigServerIntf>(
            ctx, objectPath, properties);

        auto config = co_await PortFactoryIntf::getConfig(
            ctx, std::string(objectPath), USBPortConfigServerIntf::interface);

        if (!shouldPass)
        {
            VerifyConfig(config, nullptr);
            co_return;
        }

        auto usbConfig = static_cast<PortConfigIntf::TestPortConfig&>(*config);

        VerifyConfig(usbConfig.name, properties.name);
        VerifyConfig(usbConfig.portMode, PortConfigIntf::PortMode::rs485);
        VerifyConfig(usbConfig.baudRate, properties.baud_rate);
        VerifyConfig(usbConfig.rtsDelay, properties.rts_delay);
        VerifyConfig(usbConfig.address, properties.device_address);
        VerifyConfig(usbConfig.port, properties.port);
        VerifyConfig(usbConfig.interface, properties.device_interface);

        getPortConfigPassed = true;

        co_return;
    }
};

TEST_F(PortTest, TestUpdateConfig)
{
    PortConfigIntf::Config config = {};
    auto res = PortConfigIntf::updateBaseConfig(config, properties);
    EXPECT_TRUE(res) << "Failed to update config";

    EXPECT_EQ(config.name, properties.name);
    EXPECT_EQ(config.portMode, PortConfigIntf::PortMode::rs485);
    EXPECT_EQ(config.baudRate, properties.baud_rate);
    EXPECT_EQ(config.rtsDelay, properties.rts_delay);
}

TEST_F(PortTest, TestGetUSBPortConfigSucess)
{
    using InventoryIntf =
        sdbusplus::client::xyz::openbmc_project::inventory::Item<>;
    sdbusplus::server::manager_t manager{ctx, InventoryIntf::namespace_path};

    const USBPortConfigServerIntf::properties_t properties = {
        .type = "USBPort",
        .name = "USBPort1",
        .device_address = "0xa",
        .device_interface = 1,
        .port = 1,
        .mode = "RS485",
        .baud_rate = 115200,
        .rts_delay = 100};

    ctx.spawn(TestGetUSBPortConfig(properties, true));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.request_name(entity_manager::EntityManagerInterface::serviceName);
    ctx.run();

    EXPECT_EQ(getPortConfigPassed, true);
}

TEST_F(PortTest, TestGetUSBPortConfigFailureForInvalidPortMode)
{
    using InventoryIntf =
        sdbusplus::client::xyz::openbmc_project::inventory::Item<>;
    sdbusplus::server::manager_t manager{ctx, InventoryIntf::namespace_path};

    const USBPortConfigServerIntf::properties_t properties = {
        .type = "USBPort",
        .name = "USBPort1",
        .device_address = "0xa",
        .device_interface = 1,
        .port = 1,
        .mode = "RSXXX", // Invalid port mode
        .baud_rate = 115200,
        .rts_delay = 100};

    ctx.spawn(TestGetUSBPortConfig(properties, false));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.request_name(entity_manager::EntityManagerInterface::serviceName);
    ctx.run();

    EXPECT_EQ(getPortConfigPassed, false);
}

TEST_F(PortTest, TestReadHoldingRegisterSuccess)
{
    PortConfigIntf::Config config = {};
    auto res = PortConfigIntf::updateBaseConfig(config, properties);
    EXPECT_TRUE(res) << "Failed to update config";

    MockPort port(ctx, config, clientDevicePath);

    ctx.spawn(serverTester->processRequests());

    ctx.spawn(TestHoldingRegisters(
        config, port, TestIntf::testSuccessReadHoldingRegisterOffset, true));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}

TEST_F(PortTest, TestReadHoldingRegisterFailure)
{
    PortConfigIntf::Config config = {};
    auto res = PortConfigIntf::updateBaseConfig(config, properties);
    EXPECT_TRUE(res) << "Failed to update config";

    MockPort port(ctx, config, clientDevicePath);

    ctx.spawn(serverTester->processRequests());

    ctx.spawn(TestHoldingRegisters(
        config, port, TestIntf::testFailureReadHoldingRegister, false));

    ctx.spawn(sdbusplus::async::sleep_for(ctx, 1s) |
              sdbusplus::async::execution::then([&]() { ctx.request_stop(); }));

    ctx.run();
}
