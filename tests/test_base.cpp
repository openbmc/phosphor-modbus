#include "test_base.hpp"

#include <unistd.h>

static constexpr auto maxRetries = 50;
static constexpr auto retryIntervalUs = 100000; // 100ms

BaseTest::BaseTest(const char* clientDevicePath, const char* serverDevicePath,
                   const char* serviceName)
{
    std::string socatCmd = std::format(
        "socat -x -v -d -d pty,link={},rawer,echo=0,parenb,{} pty,link={},rawer,echo=0,parenb,{} & echo $!",
        serverDevicePath, strBaudeRate, clientDevicePath, strBaudeRate);

    // Start socat in the background and capture its PID
    FILE* fp = popen(socatCmd.c_str(), "r");
    EXPECT_NE(fp, nullptr) << "Failed to start socat: " << strerror(errno);
    EXPECT_GT(fscanf(fp, "%d", &socatPid), 0);
    pclose(fp);

    // Wait for socat to create the device symlinks
    for (int i = 0; i < maxRetries; i++)
    {
        if (access(clientDevicePath, F_OK) == 0 &&
            access(serverDevicePath, F_OK) == 0)
        {
            break;
        }
        usleep(retryIntervalUs);
    }

    fdClient = open(clientDevicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
    EXPECT_NE(fdClient, -1) << "Failed to open serial port " << clientDevicePath
                            << " with error: " << strerror(errno);

    fdServer = open(serverDevicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
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
    if (socatPid > 0)
    {
        kill(socatPid, SIGTERM);
        // Wait for socat to fully exit so it cleans up its symlinks
        // before the next test starts a new socat instance.
        for (int i = 0; i < maxRetries; i++)
        {
            if (kill(socatPid, 0) != 0)
            {
                break;
            }
            usleep(retryIntervalUs);
        }
    }
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
