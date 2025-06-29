#include "modbus/modbus.hpp"
#include "modbus_server_tester.hpp"

#include <fcntl.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace std::literals;

namespace RTUIntf = phosphor::modbus::rtu;
using ModbusIntf = RTUIntf::Modbus;
namespace TestIntf = phosphor::modbus::test;

class ModbusTest : public ::testing::Test
{
  public:
    static constexpr const char* clientDevicePath = "/dev/ttyV0";
    static constexpr const char* serverDevicePath = "/dev/ttyV1";
    static constexpr const auto defaultBaudeRate = "b115200";
    int socat_pid = -1;
    static constexpr uint8_t deviceAddress = 10;
    std::optional<sdbusplus::async::context> ctx{std::in_place};
    std::unique_ptr<ModbusIntf> modbus;
    int fdClient = -1;
    std::unique_ptr<TestIntf::ServerTester> serverTester;
    int fdServer = -1;

    ModbusTest()
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

        modbus = std::make_unique<ModbusIntf>(*ctx, fdClient, 115200, 0);

        fdServer = open(serverDevicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
        EXPECT_NE(fdServer, -1)
            << "Failed to open serial port " << serverDevicePath
            << " with error: " << strerror(errno);

        serverTester = std::make_unique<TestIntf::ServerTester>(*ctx, fdServer);
    }

    ~ModbusTest() noexcept override
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
        ctx.reset();
        kill(socat_pid, SIGTERM);
    }

    void SetUp() override
    {
        ctx->spawn(serverTester->processRequests());
    }

    auto TestHoldingRegisters(uint16_t registerOffset, bool res)
        -> sdbusplus::async::task<void>
    {
        std::cout << "TestHoldingRegisters() start" << std::endl;

        std::vector<uint16_t> registers(2);

        auto ret = co_await modbus->readHoldingRegisters(
            deviceAddress, registerOffset, registers);

        EXPECT_EQ(ret, res) << "Failed to read holding registers";

        co_return;
    }
};

TEST_F(ModbusTest, TestReadHoldingRegisterSuccess)
{
    ctx->spawn(TestHoldingRegisters(
        TestIntf::testSuccessReadModbusHoldingRegister, true));

    ctx->spawn(
        sdbusplus::async::sleep_for(*ctx, 1s) |
        sdbusplus::async::execution::then([&]() { ctx->request_stop(); }));

    ctx->run();
}

TEST_F(ModbusTest, TestReadHoldingRegisterFailure)
{
    ctx->spawn(TestHoldingRegisters(
        TestIntf::testFailureReadModbusHoldingRegister, false));

    ctx->spawn(
        sdbusplus::async::sleep_for(*ctx, 1s) |
        sdbusplus::async::execution::then([&]() { ctx->request_stop(); }));

    ctx->run();
}
