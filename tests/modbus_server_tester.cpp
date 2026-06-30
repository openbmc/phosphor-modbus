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
constexpr uint8_t writeMultipleRegistersFunctionCode = 0x10;
constexpr uint8_t writeMultipleRegistersErrorFunctionCode = 0x90;

ServerTester::ServerTester(sdbusplus::async::context& ctx, int fd) :
    fd(fd), fdioInstance(ctx, fd), mutex("TestMutex")
{}

auto ServerTester::processRequestsInternal() -> void
{
    MessageIntf request;

    auto ret = read(fd, request.raw.data(), request.raw.size());
    // Request message need to be at least 4 bytes long - address(1),
    // function code(1), ..., CRC(2)
    if (ret < 4)
    {
        error("Invalid Server message size {SIZE}, drop it", "SIZE", ret);
        return;
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
        return;
    }

    // Segmented response
    ret = write(fd, response.raw.data(), response.len - 2);
    if (ret < 0)
    {
        error("Failed to send 1st segment response {ERROR}", "ERROR",
              strerror(errno));
        return;
    }

    debug("First segment sent successfully");

    ret = write(fd, response.raw.data() + response.len - 2, 2);
    if (ret < 0)
    {
        error("Failed to send 2nd segment response {ERROR}", "ERROR",
              strerror(errno));
        return;
    }

    debug("Second segment sent successfully");
}

auto ServerTester::processRequests() -> void
{
    fd_set readFds;
    FD_ZERO(&readFds);
    FD_SET(fd, &readFds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 500000;

    int readyFds = select(fd + 1, &readFds, nullptr, nullptr, &timeout);

    if (readyFds == -1)
    {
        // Error handling
        error("Error in select {ERROR}", "ERROR", strerror(errno));
    }
    else if (readyFds == 0)
    {
        // Timeout occurred, no data to read
        warning("Timeout occurred, no data to read");
    }
    else
    {
        if (FD_ISSET(fd, &readFds))
        {
            // Data is available to read
            processRequestsInternal();
        }
    }
}

void ServerTester::processMessage(MessageIntf& request, size_t requestSize,
                                  MessageIntf& response,
                                  bool& segmentedResponse)
{
    EXPECT_EQ(request.address, testDeviceAddress) << "Invalid device address";

    totalRequestCount++;

    switch (request.functionCode)
    {
        case readHoldingRegistersFunctionCode:
            processReadHoldingRegisters(request, requestSize, response,
                                        segmentedResponse);
            break;
        case writeMultipleRegistersFunctionCode:
            processWriteMultipleRegisters(request, requestSize, response);
            break;
        default:
            FAIL() << "Server received unknown request function code "
                   << request.functionCode;
            break;
    }
}

static inline void checkRequestSize(size_t requestSize, size_t expectedSize)
{
    EXPECT_EQ(requestSize, expectedSize) << "Invalid request size";
}

void ServerTester::buildErrorResponse(MessageIntf& request,
                                      MessageIntf& response,
                                      RTUIntf::ModbusExceptionCode code)
{
    response << request.raw[0] << (uint8_t)readHoldingRegistersErrorFunctionCode
             << uint8_t(code);
    response.appendCRC();
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

    if (registerOffset == testFailureReadHoldingRegister)
    {
        buildErrorResponse(request, response,
                           RTUIntf::ModbusExceptionCode::illegalFunctionCode);
    }
    else if (registerOffset + registerCount > testIllegalDataAddressRegister &&
             registerOffset <= testIllegalDataAddressRegister)
    {
        // The requested register range overlaps the illegal data address
        buildErrorResponse(request, response,
                           RTUIntf::ModbusExceptionCode::illegalDataAddress);
    }
    else if (registerOffset == testFlakyReadHoldingRegisterOffset)
    {
        processFlakyRegister(request, registerCount, response);
    }
    else
    {
        processSuccessfulRead(request, registerOffset, registerCount, response,
                              segmentedResponse);
    }
}

void ServerTester::processSuccessfulRead(
    MessageIntf& request, uint16_t registerOffset, uint16_t registerCount,
    MessageIntf& response, bool& segmentedResponse)
{
    auto expectedResponseIter = testReadHoldingRegisterMap.find(registerOffset);
    if (expectedResponseIter == testReadHoldingRegisterMap.end())
    {
        FAIL() << "Invalid register offset:" << registerOffset;
        return;
    }

    checkRequestSize(registerCount, std::get<0>(expectedResponseIter->second));

    auto& expectedResponse = std::get<1>(expectedResponseIter->second);

    response << request.raw[0] << request.raw[1] << uint8_t(2 * registerCount);
    for (size_t i = 0; i < registerCount; i++)
    {
        response << uint16_t(expectedResponse[i]);
    }
    response.appendCRC();

    segmentedResponse =
        (registerOffset == testSuccessReadHoldingRegisterSegmentedOffset);
}

void ServerTester::processWriteMultipleRegisters(
    MessageIntf& request, size_t requestSize, MessageIntf& response)
{
    // NOTE: This code deliberately avoids using any packing helpers from
    // message.hpp. This ensures that message APIs are tested as intended on the
    // client side.
    uint16_t registerOffset = request.raw[2] << 8 | request.raw[3];
    uint16_t registerCount = request.raw[4] << 8 | request.raw[5];
    uint8_t byteCount = request.raw[6];

    // addr(1), func(1), offset(2), count(2), bytecount(1), data, crc(2)
    const size_t expectedRequestSize = 9 + (2 * registerCount);
    checkRequestSize(requestSize, expectedRequestSize);
    EXPECT_EQ(byteCount, 2 * registerCount) << "Invalid byte count";

    if (registerOffset == testFailureWriteMultipleRegistersOffset)
    {
        response << request.raw[0]
                 << (uint8_t)writeMultipleRegistersErrorFunctionCode
                 << uint8_t(RTUIntf::ModbusExceptionCode::illegalDataAddress);
        response.appendCRC();
        return;
    }

    if (registerOffset == testFlakyWriteMultipleRegistersOffset)
    {
        flakyWriteRequestCount++;
        if (flakyWriteRequestCount % 2 == 1)
        {
            // Odd (first) requests fail, retry succeeds
            response << request.raw[0]
                     << (uint8_t)writeMultipleRegistersErrorFunctionCode
                     << uint8_t(
                            RTUIntf::ModbusExceptionCode::illegalFunctionCode);
            response.appendCRC();
            return;
        }
    }
    else
    {
        // Verify the streamed register data matches what the client encoded
        for (size_t i = 0; i < registerCount; i++)
        {
            uint16_t value = request.raw[7 + (2 * i)] << 8 |
                             request.raw[8 + (2 * i)];
            EXPECT_EQ(value, testWriteMultipleRegistersData[i])
                << "Invalid register data at index " << i;
        }
    }

    // Success: echo back addr, func, offset and count
    response << request.raw[0] << request.raw[1] << request.raw[2]
             << request.raw[3] << request.raw[4] << request.raw[5];
    response.appendCRC();
}

void ServerTester::processFlakyRegister(
    MessageIntf& request, uint16_t registerCount, MessageIntf& response)
{
    flakyRegisterRequestCount++;
    if (flakyRegisterRequestCount % 2 == 0)
    {
        // Even requests fail
        response << request.raw[0]
                 << (uint8_t)readHoldingRegistersErrorFunctionCode
                 << uint8_t(RTUIntf::ModbusExceptionCode::illegalFunctionCode);
    }
    else
    {
        // Odd requests succeed
        response << request.raw[0] << request.raw[1]
                 << uint8_t(2 * registerCount);
        for (size_t i = 0; i < registerCount; i++)
        {
            response << uint16_t(0x0050);
        }
    }
    response.appendCRC();
}

} // namespace phosphor::modbus::test
