#include "modbus_server_tester.hpp"

#include <fcntl.h>

#include <sdbusplus/async.hpp>

#include <gtest/gtest.h>

namespace TestIntf = phosphor::modbus::test;

class BaseTest : public ::testing::Test
{
  public:
    static constexpr auto baudRate = 115200;

    BaseTest(const char* clientPathPrefix, const char* serverPathPrefix,
             const char* serviceName);
    ~BaseTest() noexcept override;
    void SetUp() override;
    void TearDown() override;

    sdbusplus::async::context ctx;
    int fdClient = -1;
    // Unique per-test device paths (prefix + test name) to avoid socat races
    // between tests.
    std::string clientDevicePath;
    std::string serverDevicePath;

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
