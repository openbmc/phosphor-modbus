#include "modbus_server_tester.hpp"

#include "modbus/modbus.hpp"
#include "modbus/modbus_commands.hpp"
#include "modbus/modbus_exception.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>

#include <gtest/gtest.h>

namespace phosphor::modbus::test
{

PHOSPHOR_LOG2_USING;

namespace RTUIntf = phosphor::modbus::rtu;
using namespace std::literals;

constexpr uint8_t readHoldingRegistersFunctionCode = 0x3;
constexpr uint8_t readHoldingRegistersErrorFunctionCode = 0x83;

ServerTester::ServerTester(sdbusplus::async::context& ctx, int fd) :
    fd(fd), fdioInstance(ctx, fd)
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

    // Segmented response
    ret = write(fd, response.raw.data(), response.len - 2);
    if (ret < 0)
    {
        error("Failed to send 1st segment response {ERROR}", "ERROR",
              strerror(errno));
        co_return;
    }

    debug("First segment sent successfully");

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
    EXPECT_EQ(request.address, testDeviceAddress) << "Invalid device address";

    switch (request.functionCode)
    {
        case readHoldingRegistersFunctionCode:
            processReadHoldingRegisters(request, requestSize, response,
                                        segmentedResponse);
            break;
        default:
            FAIL() << "Server received unknown request function code "
                   << request.functionCode;
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

    // NOTE: This code deliberately avoids using any packing helpers from
    // message.hpp. This ensures that message APIs are tested as intended on the
    // client side.
    uint16_t registerOffset = request.raw[2] << 8 | request.raw[3];
    uint16_t registerCount = request.raw[4] << 8 | request.raw[5];

    EXPECT_EQ(registerCount, testSuccessReadHoldingRegisterCount);

    if (registerOffset == testSuccessReadHoldingRegisterOffset ||
        registerOffset == testSuccessReadHoldingRegisterSegmentedOffset)
    {
        response << request.raw[0] << request.raw[1]
                 << uint8_t(2 * registerCount)
                 << uint16_t(testSuccessReadHoldingRegisterResponse[0])
                 << uint16_t(testSuccessReadHoldingRegisterResponse[1]);
        response.appendCRC();
        segmentedResponse =
            (registerOffset == testSuccessReadHoldingRegisterSegmentedOffset);
    }
    else if (registerOffset == testFailureReadHoldingRegister)
    {
        response << request.raw[0]
                 << (uint8_t)readHoldingRegistersErrorFunctionCode
                 << uint8_t(RTUIntf::ModbusExceptionCode::illegalFunctionCode);
        response.appendCRC();
    }
    else
    {
        FAIL() << "Invalid register offset:" << registerOffset;
    }
}

} // namespace phosphor::modbus::test
