#include "modbus_server_tester.hpp"

#include "modbus/modbus.hpp"
#include "modbus/modbus_commands.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>

#include <gtest/gtest.h>

namespace phosphor::modbus::test
{

PHOSPHOR_LOG2_USING;

using namespace std::literals;

constexpr uint8_t readHoldingRegistersFunctionCode = 0x3;
constexpr uint8_t readHoldingRegistersErrorFunctionCode = 0x83;

ServerTester::ServerTester(sdbusplus::async::context& ctx, int fd) :
    ctx(ctx), fd(fd), fdioInstance(ctx, fd)
{}

auto ServerTester::processRequests() -> sdbusplus::async::task<void>
{
    MessageIntf request;
    co_await fdioInstance.next();
    auto ret = read(fd, request.raw.data(), request.raw.size());
    // Request message need to be at least 4 bytes long - address(1),
    // function code(1), ..., CRC(2)
    if (ret < 4)
    {
        error("Invalid Server message size {SIZE}, drop it", "SIZE", ret);
        co_return;
    }

    MessageIntf response;
    bool segmentedResponse = false;
    processMessage(request, ret, response, segmentedResponse);

    if (!segmentedResponse)
    {
        ret = write(fd, response.raw.data(), response.len);
        if (ret < 0)
        {
            error("Failed to send response {ERROR}", "ERROR", strerror(errno));
        }
        co_return;
    }

    // segmented response
    ret = write(fd, response.raw.data(), response.len - 2);
    if (ret < 0)
    {
        error("Failed to send 1st segment response {ERROR}", "ERROR",
              strerror(errno));
        co_return;
    }

    debug("First segment sent successfully");

    co_await sdbusplus::async::sleep_for(ctx, std::chrono::seconds(1));

    // wait for next message
    ret = write(fd, response.raw.data() + response.len - 2, 2);
    if (ret < 0)
    {
        error("Failed to send 2nd segment response {ERROR}", "ERROR",
              strerror(errno));
        co_return;
    }

    debug("Second segment sent successfully");

    co_return;
}

void ServerTester::processMessage(MessageIntf& request, size_t requestSize,
                                  MessageIntf& response,
                                  bool& segmentedResponse)
{
    switch (request.functionCode)
    {
        case readHoldingRegistersFunctionCode:
            processReadHoldingRegisters(request, requestSize, response,
                                        segmentedResponse);
            break;
        default:
            FAIL() << "Server received unknown request";
            break;
    }
}

void ServerTester::processReadHoldingRegisters(
    MessageIntf& request, size_t requestSize, MessageIntf& response,
    bool& segmentedResponse)
{
    constexpr size_t expectedRequestSize = 8;

    if (requestSize != expectedRequestSize)
    {
        FAIL() << "Invalid readHoldingRegisters request size:" << requestSize
               << ", drop it";
        return;
    }

    uint16_t registerOffset = request.raw[2] << 8 | request.raw[3];
    uint16_t registerCount = request.raw[4] << 8 | request.raw[5];

    if (registerOffset == testSuccessReadModbusHoldingRegister ||
        registerOffset == testSuccessReadModbusHoldingRegisterSegmented)
    {
        response << request.raw[0] << request.raw[1]
                 << uint8_t(2 * registerCount) << uint16_t(0x1234)
                 << uint16_t(0x5678);
        response.appendCRC();
        segmentedResponse =
            (registerOffset == testSuccessReadModbusHoldingRegisterSegmented);
    }
    else if (registerOffset == testFailureReadModbusHoldingRegister)
    {
        response << request.raw[0]
                 << (uint8_t)readHoldingRegistersErrorFunctionCode
                 << uint8_t(0x01);
        response.appendCRC();
    }
}

} // namespace phosphor::modbus::test
