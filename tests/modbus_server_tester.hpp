#pragma once

#include "modbus/modbus_exception.hpp"
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
const std::vector<uint16_t> testSuccessReadHoldingRegisterResponse = {
    0x1234, 0x5678};
constexpr uint16_t testFailureReadHoldingRegister = 0x0105;
constexpr uint16_t testIllegalDataAddressRegister = 0x0108;
constexpr uint16_t testFlakyReadHoldingRegisterOffset = 0x0106;
constexpr uint16_t testFlakyReadHoldingRegisterCount = 0x1;

// Device Inventory Testing Constants
constexpr uint16_t testReadHoldingRegisterModelOffset = 0x0112;
constexpr uint16_t testReadHoldingRegisterModelCount = 0x8;
const std::vector<uint16_t> testReadHoldingRegisterModel = {
    0x5244, 0x4630, 0x3430, 0x4453, 0x5335, 0x3139, 0x0000, 0x3000};
constexpr std::string testReadHoldingRegisterModelStr = "RDF040DSS519";

// Device Sensors Testing Constants
constexpr uint16_t testReadHoldingRegisterTempCount = 0x1;
constexpr uint16_t testReadHoldingRegisterTempUnsignedOffset = 0x0113;
const std::vector<uint16_t> testReadHoldingRegisterTempUnsigned = {
    0x0050}; // 80.0
constexpr uint16_t testReadHoldingRegisterTempSignedOffset = 0x0114;
const std::vector<uint16_t> testReadHoldingRegisterTempSigned = {
    0xFFB0}; // -80.0

// Contiguous sensor registers for span merge testing (0x0120..0x0121)
constexpr uint16_t testReadHoldingRegisterSpanSensor1Offset = 0x0120;
constexpr uint16_t testReadHoldingRegisterSpanSensor2Offset = 0x0121;
constexpr uint16_t testReadHoldingRegisterSpanMergedCount = 0x2;
const std::vector<uint16_t> testReadHoldingRegisterSpanMerged = {
    0x0050, 0x00C8};

// Distant sensor register for testing separate spans
constexpr uint16_t testReadHoldingRegisterDistantOffset = 0x0200;
constexpr uint16_t testReadHoldingRegisterDistantCount = 0x1;
const std::vector<uint16_t> testReadHoldingRegisterDistant = {0x00C8}; // 200

// Device Firmware Testing Constants
constexpr uint16_t testReadHoldingRegisterFirmwareVersionOffset = 0x0115;
constexpr uint16_t testReadHoldingRegisterFirmwareVersionCount = 0x2;
const std::vector<uint16_t> testReadHoldingRegisterFirmwareVersion = {
    0x5244, 0x4630};
constexpr std::string testReadHoldingRegisterFirmwareVersionStr = "RDF0";

// Device Event Testing Constants
constexpr uint16_t testReadHoldingRegisterEventCount = 0x1;
constexpr uint16_t testReadHoldingRegisterEventOffset = 0x0116;
const std::vector<uint16_t> testReadHoldingRegisterEvent = {
    0x0001}; // Event is enabled

static const std::map<uint16_t, std::tuple<uint16_t, std::vector<uint16_t>>>
    testReadHoldingRegisterMap = {
        {testSuccessReadHoldingRegisterOffset,
         {testSuccessReadHoldingRegisterCount,
          testSuccessReadHoldingRegisterResponse}},
        {testSuccessReadHoldingRegisterSegmentedOffset,
         {testSuccessReadHoldingRegisterCount,
          testSuccessReadHoldingRegisterResponse}},
        {testReadHoldingRegisterModelOffset,
         {testReadHoldingRegisterModelCount, testReadHoldingRegisterModel}},
        {testReadHoldingRegisterTempUnsignedOffset,
         {testReadHoldingRegisterTempCount,
          testReadHoldingRegisterTempUnsigned}},
        {testReadHoldingRegisterTempSignedOffset,
         {testReadHoldingRegisterTempCount, testReadHoldingRegisterTempSigned}},
        {testReadHoldingRegisterSpanSensor1Offset,
         {testReadHoldingRegisterSpanMergedCount,
          testReadHoldingRegisterSpanMerged}},
        {testReadHoldingRegisterDistantOffset,
         {testReadHoldingRegisterDistantCount, testReadHoldingRegisterDistant}},
        {testReadHoldingRegisterFirmwareVersionOffset,
         {testReadHoldingRegisterFirmwareVersionCount,
          testReadHoldingRegisterFirmwareVersion}},
        {testReadHoldingRegisterEventOffset,
         {testReadHoldingRegisterEventCount, testReadHoldingRegisterEvent}}};

class ServerTester
{
  public:
    explicit ServerTester(sdbusplus::async::context& ctx, int fd);

    auto processRequests() -> void;

    std::atomic<uint32_t> totalRequestCount{0};

  private:
    auto processMessage(MessageIntf& request, size_t requestSize,
                        MessageIntf& response, bool& segmentedResponse) -> void;

    auto processRequestsInternal() -> void;

    auto processReadHoldingRegisters(MessageIntf& request, size_t requestSize,
                                     MessageIntf& response,
                                     bool& segmentedResponse) -> void;

    auto processFlakyRegister(MessageIntf& request, uint16_t registerCount,
                              MessageIntf& response) -> void;

    auto buildErrorResponse(MessageIntf& request, MessageIntf& response,
                            phosphor::modbus::rtu::ModbusExceptionCode code)
        -> void;

    auto processSuccessfulRead(MessageIntf& request, uint16_t registerOffset,
                               uint16_t registerCount, MessageIntf& response,
                               bool& segmentedResponse) -> void;

    int fd;
    sdbusplus::async::fdio fdioInstance;
    sdbusplus::async::mutex mutex;
    uint32_t flakyRegisterRequestCount = 0;
};
} // namespace phosphor::modbus::test
