#include "test_base.hpp"

#include <unistd.h>

BaseTest::BaseTest(const char* clientPathPrefix, const char* serverPathPrefix,
                   const char* serviceName)
{
    auto testInfo = ::testing::UnitTest::GetInstance()->current_test_info();
    clientDevicePath = std::format("{}_{}", clientPathPrefix, testInfo->name());
    serverDevicePath = std::format("{}_{}", serverPathPrefix, testInfo->name());

    std::string socatCmd = std::format(
        "socat -x -v -d -d pty,link={},rawer,echo=0,parenb,{} pty,link={},rawer,echo=0,parenb,{} & echo $!",
        serverDevicePath, strBaudeRate, clientDevicePath, strBaudeRate);

    // Start socat in the background and capture its PID
    FILE* fp = popen(socatCmd.c_str(), "r");
    EXPECT_NE(fp, nullptr) << "Failed to start socat: " << strerror(errno);
    EXPECT_GT(fscanf(fp, "%d", &socatPid), 0);
    pclose(fp);

    // Wait for socat to create the device symlinks
    constexpr auto maxRetries = 20;
    constexpr auto retryIntervalUs = 50000; // 50ms
    for (int i = 0; i < maxRetries; i++)
    {
        if (access(clientDevicePath.c_str(), F_OK) == 0 &&
            access(serverDevicePath.c_str(), F_OK) == 0)
        {
            break;
        }
        usleep(retryIntervalUs);
    }

    fdClient = open(clientDevicePath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    EXPECT_NE(fdClient, -1) << "Failed to open serial port " << clientDevicePath
                            << " with error: " << strerror(errno);

    fdServer = open(serverDevicePath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    EXPECT_NE(fdServer, -1) << "Failed to open serial port " << serverDevicePath
                            << " with error: " << strerror(errno);

    ctx.request_name(serviceName);

    serverTester = std::make_unique<TestIntf::ServerTester>(ctx, fdServer);
}

BaseTest::~BaseTest() noexcept
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
    kill(socatPid, SIGTERM);
}

void BaseTest::SetUp()
{
    serverThread = std::thread(&BaseTest::ServerRequestHandler, this);
}

void BaseTest::TearDown()
{
    exitThread.store(true);
    if (serverThread.joinable())
    {
        serverThread.join();
    }
}

void BaseTest::ServerRequestHandler()
{
    while (!exitThread.load())
    {
        serverTester->processRequests();
    }
}
