#pragma once

#include "modbus/modbus_message.hpp"

#include <sdbusplus/async.hpp>

using MessageBase = phosphor::modbus::rtu::Message;

namespace phosphor::modbus::test
{

class MessageIntf : public MessageBase
{
    friend class ServerTester;
};

static constexpr uint8_t testDeviceAddress = 0xa;
constexpr uint16_t testSuccessReadHoldingRegisterOffset = 0x0102;
constexpr uint16_t testSuccessReadHoldingRegisterCount = 0x2;
constexpr uint16_t testSuccessReadHoldingRegisterSegmentedOffset = 0x0103;
constexpr std::array<uint16_t, testSuccessReadHoldingRegisterCount>
    testSuccessReadHoldingRegisterResponse = {0x1234, 0x5678};
constexpr uint16_t testFailureReadHoldingRegister = 0x0105;

class ServerTester
{
  public:
    explicit ServerTester(sdbusplus::async::context& ctx, int fd);

    auto processRequests() -> sdbusplus::async::task<void>;

  private:
    void processMessage(MessageIntf& request, size_t requestSize,
                        MessageIntf& response, bool& segmentedResponse);

    void processReadHoldingRegisters(MessageIntf& request, size_t requestSize,
                                     MessageIntf& response,
                                     bool& segmentedResponse);

    int fd;
    sdbusplus::async::fdio fdioInstance;
};
} // namespace phosphor::modbus::test
