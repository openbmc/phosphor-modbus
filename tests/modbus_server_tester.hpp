#pragma once

#include "modbus/modbus_message.hpp"

#include <sdbusplus/async.hpp>

// using MessageIntf = phosphor::modbus::rtu::Message;
using MessageBase = phosphor::modbus::rtu::Message;

namespace phosphor::modbus::test
{

class MessageIntf : public MessageBase
{
    friend class ServerTester;
};

constexpr uint16_t testSuccessReadModbusHoldingRegister = 0x0102;
constexpr uint16_t testSuccessReadModbusHoldingRegisterSegmented = 0x0103;
constexpr uint16_t testFailureReadModbusHoldingRegister = 0x0105;

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
