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

// Read Holding Registers Testing Constants
static constexpr uint8_t testDeviceAddress = 0xa;
constexpr uint16_t testSuccessReadHoldingRegisterOffset = 0x0102;
constexpr uint16_t testSuccessReadHoldingRegisterCount = 0x2;
constexpr uint16_t testSuccessReadHoldingRegisterSegmentedOffset = 0x0103;
constexpr std::array<uint16_t, testSuccessReadHoldingRegisterCount>
    testSuccessReadHoldingRegisterResponse = {0x1234, 0x5678};
constexpr uint16_t testFailureReadHoldingRegister = 0x0105;

// Device Inventory Testing Constants
constexpr uint16_t testReadHoldingRegisterModelOffset = 0x0112;
constexpr uint16_t testReadHoldingRegisterModelCount = 0x8;
constexpr std::array<uint16_t, testReadHoldingRegisterModelCount>
    testReadHoldingRegisterModel = {0x5244, 0x4630, 0x3430, 0x4453,
                                    0x5335, 0x3139, 0x0000, 0x3000};
constexpr std::string testReadHoldingRegisterModelStr = "RDF040DSS519";

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
    sdbusplus::async::mutex mutex;
};
} // namespace phosphor::modbus::test
