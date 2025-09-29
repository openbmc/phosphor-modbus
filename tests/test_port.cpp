#include "modbus_server_tester.hpp"
#include "port/base_port.hpp"

#include <fcntl.h>

#include <gtest/gtest.h>

using namespace std::literals;

namespace TestIntf = phosphor::modbus::test;
namespace PortIntf = phosphor::modbus::rtu::port;
namespace PortConfigIntf = PortIntf::config;
namespace RTUIntf = phosphor::modbus::rtu;

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
