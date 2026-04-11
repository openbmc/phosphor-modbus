#include "modbus_server_tester.hpp"

#include <fcntl.h>

#include <sdbusplus/async.hpp>

#include <gtest/gtest.h>

namespace TestIntf = phosphor::modbus::test;

class BaseTest : public ::testing::Test
{
  public:
    static constexpr auto baudRate = 115200;

    BaseTest(const char* clientDevicePath, const char* serverDevicePath,
             const char* serviceName);
    ~BaseTest() noexcept override;
    void SetUp() override;
    void TearDown() override;

    sdbusplus::async::context ctx;
    int fdClient = -1;

  protected:
    std::unique_ptr<TestIntf::ServerTester> serverTester;

  private:
    void ServerRequestHandler();

    static constexpr const auto strBaudeRate = "b115200";
    int socatPid = -1;
    int fdServer = -1;
    std::atomic<bool> exitThread = false;
    std::thread serverThread;
};
