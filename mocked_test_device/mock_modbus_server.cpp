#include "modbus/modbus.hpp"
#include "modbus/modbus_commands.hpp"
#include "modbus/modbus_message.hpp"

#include <fcntl.h>

#include <sdbusplus/async.hpp>

#include <iostream>
#include <string>

namespace phosphor::modbus::test
{

class MessageIntf : public phosphor::modbus::rtu::Message
{
    friend class TestServer;
};

class TestServer
{
  public:
    explicit TestServer(sdbusplus::async::context& ctx, int fd);

  private:
    auto processRequests() -> sdbusplus::async::task<void>;
    void processMessage(MessageIntf& request, size_t requestSize,
                        MessageIntf& response);

    void processReadHoldingRegisters(MessageIntf& request, size_t requestSize,
                                     MessageIntf& response);

    sdbusplus::async::context& ctx;
    int fd;
    sdbusplus::async::fdio fdioInstance;
};

TestServer::TestServer(sdbusplus::async::context& ctx, int fd) :
    ctx(ctx), fd(fd), fdioInstance(ctx, fd)
{
    ctx.spawn(processRequests());
}

auto TestServer::processRequests() -> sdbusplus::async::task<void>
{
    while (!ctx.stop_requested())
    {
        MessageIntf request;
        co_await fdioInstance.next();
        auto ret = read(fd, request.raw.data(), request.raw.size());
        // Request message need to be at least 4 bytes long - address(1),
        // function code(1), ..., CRC(2)
        if (ret < 4)
        {
            std::cerr << "Invalid Server message size:" << ret << ", drop it"
                      << std::endl;
            continue;
        }

        MessageIntf response;
        processMessage(request, ret, response);

        ret = write(fd, response.raw.data(), response.len);
        if (ret < 0)
        {
            std::cerr << "Failed to send response" << std::endl;
        }
    }
}

void TestServer::processMessage(MessageIntf& request, size_t requestSize,
                                MessageIntf& response)
{
    constexpr uint8_t readHoldingRegistersFunctionCode = 0x3;

    switch (request.functionCode)
    {
        case readHoldingRegistersFunctionCode:
            processReadHoldingRegisters(request, requestSize, response);
            break;
        default:
            std::cerr << "Server received unknown request" << std::endl;
            break;
    }
}

void TestServer::processReadHoldingRegisters(
    MessageIntf& request, size_t requestSize, MessageIntf& response)
{
    uint16_t registerOffset = request.raw[2] << 8 | request.raw[3];
    uint16_t registerCount = request.raw[4] << 8 | request.raw[5];

    std::cout << "Received readHoldingRegisters request with size:"
              << requestSize << ", registerOffset:" << registerOffset
              << ", registerCount:" << registerCount << std::endl;

    response << request.raw[0] << request.raw[1] << uint8_t(2 * registerCount);
    for (int i = 0; i < registerCount; i++)
    {
        constexpr uint16_t readHoldingRegisterResponse = 0x4142;
        response << uint16_t(readHoldingRegisterResponse);
    }
    response.appendCRC();
}

} // namespace phosphor::modbus::test

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <integer_value>" << std::endl;
        return 1;
    }

    int interfaceValue = 1;
    try
    {
        interfaceValue = std::stoi(argv[1]);
    }
    catch (const std::invalid_argument& e)
    {
        std::cerr << "Invalid argument: " << e.what() << std::endl;
        return 1;
    }
    catch (const std::out_of_range& e)
    {
        std::cerr << "Argument out of range: " << e.what() << std::endl;
        return 1;
    }

    using TestServerIntf = phosphor::modbus::test::TestServer;
    std::string devicePathStr =
        std::string("/dev/ttyV") + std::to_string(interfaceValue);
    const char* serverDevicePath = devicePathStr.c_str();
    std::cout << "Starting at device path" << serverDevicePath << std::endl;
    constexpr auto path = "/xyz/openbmc_project";
    constexpr auto serviceName = "xyz.openbmc_project.ModbusRTUTestServer";
    sdbusplus::async::context ctx;

    auto fdServer = open(serverDevicePath, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fdServer == -1)
    {
        std::cerr << "Failed to open serial port " << serverDevicePath
                  << " with error: " << strerror(errno) << std::endl;
        return 1;
    }

    std::cout << "Creating Modbus RTU test server at " << path << std::endl;
    TestServerIntf server{ctx, fdServer};

    ctx.request_name(serviceName);

    ctx.run();
    return 0;
}
