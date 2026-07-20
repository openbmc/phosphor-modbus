#pragma once

#include "modbus/modbus_exception.hpp"
#include "modbus/modbus_message.hpp"

#include <atomic>
#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <vector>

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

// Write Multiple Registers Testing Constants
constexpr uint16_t testSuccessWriteMultipleRegistersOffset = 0x0500;
constexpr uint16_t testSuccessWriteMultipleRegistersCount = 0x2;
const std::vector<uint16_t> testWriteMultipleRegistersData = {0x1122, 0x3344};
constexpr uint16_t testFailureWriteMultipleRegistersOffset = 0x0501;
constexpr uint16_t testFlakyWriteMultipleRegistersOffset = 0x0502;

// Config register write testing (device writes UnixTime here)
constexpr uint16_t testConfigWriteRegisterOffset = 0x0503;
constexpr uint16_t testConfigWriteRegisterCount = 0x2;

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

// Null-padded string register for probe testing: "AB" followed by nulls
constexpr uint16_t testReadHoldingRegisterNullPaddedOffset = 0x0300;
constexpr uint16_t testReadHoldingRegisterNullPaddedCount = 0x4;
const std::vector<uint16_t> testReadHoldingRegisterNullPadded = {
    0x4142, 0x0000, 0x0000, 0x0000}; // "AB" + 6 null bytes

// Integer inventory register for testing InventoryFormat::integer
constexpr uint16_t testReadHoldingRegisterIntInventoryOffset = 0x0400;
constexpr uint16_t testReadHoldingRegisterIntInventoryCount = 0x1;
const std::vector<uint16_t> testReadHoldingRegisterIntInventory = {42};

// Device Metric Testing Constants
constexpr uint16_t testReadHoldingRegisterMetricOffset = 0x0130;
constexpr uint16_t testReadHoldingRegisterMetricCount = 0x1;
const std::vector<uint16_t> testReadHoldingRegisterMetric = {0x012C}; // 300

// Device Firmware Testing Constants
constexpr uint16_t testReadHoldingRegisterFirmwareVersionOffset = 0x0115;
constexpr uint16_t testReadHoldingRegisterFirmwareVersionCount = 0x2;
const std::vector<uint16_t> testReadHoldingRegisterFirmwareVersion = {
    0x5244, 0x4630};
constexpr std::string testReadHoldingRegisterFirmwareVersionStr = "RDF0";

// Device Integer Firmware Testing Constants
constexpr uint16_t testReadHoldingRegisterFirmwareIntVersionOffset = 0x0117;
constexpr uint16_t testReadHoldingRegisterFirmwareIntVersionCount = 0x2;
const std::vector<uint16_t> testReadHoldingRegisterFirmwareIntVersion = {
    0x0001, 0x0002};
constexpr std::string testReadHoldingRegisterFirmwareIntVersionStr = "65538";

// Device Float32 Sensor Testing Constants
constexpr uint16_t testReadHoldingRegisterFloat32Offset = 0x0119;
constexpr uint16_t testReadHoldingRegisterFloat32Count = 0x2;
const std::vector<uint16_t> testReadHoldingRegisterFloat32 = {
    0x422A, 0x0000}; // IEEE-754 float32: 42.5
constexpr double testReadHoldingRegisterFloat32Value = 42.5;

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
        {testReadHoldingRegisterMetricOffset,
         {testReadHoldingRegisterMetricCount, testReadHoldingRegisterMetric}},
        {testReadHoldingRegisterFirmwareVersionOffset,
         {testReadHoldingRegisterFirmwareVersionCount,
          testReadHoldingRegisterFirmwareVersion}},
        {testReadHoldingRegisterFirmwareIntVersionOffset,
         {testReadHoldingRegisterFirmwareIntVersionCount,
          testReadHoldingRegisterFirmwareIntVersion}},
        {testReadHoldingRegisterFloat32Offset,
         {testReadHoldingRegisterFloat32Count, testReadHoldingRegisterFloat32}},
        {testReadHoldingRegisterEventOffset,
         {testReadHoldingRegisterEventCount, testReadHoldingRegisterEvent}},
        {testReadHoldingRegisterNullPaddedOffset,
         {testReadHoldingRegisterNullPaddedCount,
          testReadHoldingRegisterNullPadded}},
        {testReadHoldingRegisterIntInventoryOffset,
         {testReadHoldingRegisterIntInventoryCount,
          testReadHoldingRegisterIntInventory}}};

class ServerTester
{
  public:
    explicit ServerTester(int fd);

    auto processRequests() -> void;

    /** @brief Number of read requests seen for a register offset, so tests can
     *  confirm a register's poll cadence. */
    auto readCount(uint16_t offset) const -> uint32_t;

    std::atomic<uint32_t> totalRequestCount{0};
    std::atomic<uint32_t> writeRequestCount{0};

  private:
    auto processMessage(MessageIntf& request, size_t requestSize,
                        MessageIntf& response, bool& segmentedResponse) -> void;

    auto processRequestsInternal() -> void;

    auto processReadHoldingRegisters(MessageIntf& request, size_t requestSize,
                                     MessageIntf& response,
                                     bool& segmentedResponse) -> void;

    auto processWriteMultipleRegisters(MessageIntf& request, size_t requestSize,
                                       MessageIntf& response) -> void;

    auto processFlakyRegister(MessageIntf& request, uint16_t registerCount,
                              MessageIntf& response) -> void;

    auto buildErrorResponse(MessageIntf& request, MessageIntf& response,
                            phosphor::modbus::rtu::ModbusExceptionCode code)
        -> void;

    auto processSuccessfulRead(MessageIntf& request, uint16_t registerOffset,
                               uint16_t registerCount, MessageIntf& response,
                               bool& segmentedResponse) -> void;

    int fd;
    uint32_t flakyRegisterRequestCount = 0;
    uint32_t flakyWriteRequestCount = 0;
    std::map<uint16_t, uint32_t> readCountByOffset;
};
} // namespace phosphor::modbus::test
