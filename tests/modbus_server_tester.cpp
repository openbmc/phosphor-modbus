#include "modbus_server_tester.hpp"

#include "modbus/modbus.hpp"
#include "modbus/modbus_commands.hpp"

#include <sdbusplus/async.hpp>

#include <iostream>

#include <gtest/gtest.h>

namespace phosphor::modbus::test
{

constexpr uint8_t readHoldingRegistersFunctionCode = 0x3;
constexpr uint8_t readHoldingRegistersErrorFunctionCode = 0x83;

ServerTester::ServerTester(sdbusplus::async::context& ctx, int fd) : fd(fd), fdioInstance(ctx, fd)
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
        std::cerr << "Invalid Server message size:" << ret << ", drop it"
                  << std::endl;
        co_return;
    }

    MessageIntf response;
    processMessage(request, ret, response);

    ret = write(fd, response.raw.data(), response.len);
    if (ret < 0)
    {
        std::cerr << "Failed to send response" << std::endl;
    }
}

void ServerTester::processMessage(MessageIntf& request, size_t requestSize,
                                  MessageIntf& response)
{
    switch (request.functionCode)
    {
        case readHoldingRegistersFunctionCode:
            processReadHoldingRegisters(request, requestSize, response);
            break;
        default:
            FAIL() << "Server received unknown request";
            break;
    }
}

void ServerTester::processReadHoldingRegisters(
    MessageIntf& request, size_t requestSize, MessageIntf& response)
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

    if (registerOffset == testSuccessReadModbusHoldingRegister)
    {
        response << request.raw[0] << request.raw[1]
                 << uint8_t(2 * registerCount) << uint16_t(0x1234)
                 << uint16_t(0x5678);
        response.appendCRC();
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
