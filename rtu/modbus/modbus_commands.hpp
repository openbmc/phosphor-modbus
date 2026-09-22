#pragma once

#include "modbus_message.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace phosphor::modbus::rtu
{

// A register is two bytes on the wire.
static constexpr size_t registerSize = 2;

static constexpr uint8_t ReadHoldingRegistersFunctionCode = 0x03;

class ReadHoldingRegistersRequest : public Message
{
  public:
    ReadHoldingRegistersRequest() = delete;
    ReadHoldingRegistersRequest(const ReadHoldingRegistersRequest&) = delete;
    ReadHoldingRegistersRequest& operator=(const ReadHoldingRegistersRequest&) =
        delete;
    ReadHoldingRegistersRequest(ReadHoldingRegistersRequest&&) = delete;
    ReadHoldingRegistersRequest& operator=(ReadHoldingRegistersRequest&&) =
        delete;

    explicit ReadHoldingRegistersRequest(
        uint8_t deviceAddress, uint16_t registerOffset, uint16_t registerCount);

    auto encode() -> void;

  private:
    static constexpr uint8_t commandCode = ReadHoldingRegistersFunctionCode;
    const uint8_t deviceAddress;
    const uint16_t registerOffset;
    const uint16_t registerCount;
};

static constexpr uint8_t WriteMultipleRegistersFunctionCode = 0x10;

class WriteMultipleRegistersRequest : public Message
{
  public:
    WriteMultipleRegistersRequest() = delete;
    WriteMultipleRegistersRequest(const WriteMultipleRegistersRequest&) =
        delete;
    WriteMultipleRegistersRequest& operator=(
        const WriteMultipleRegistersRequest&) = delete;
    WriteMultipleRegistersRequest(WriteMultipleRegistersRequest&&) = delete;
    WriteMultipleRegistersRequest& operator=(WriteMultipleRegistersRequest&&) =
        delete;

    explicit WriteMultipleRegistersRequest(uint8_t deviceAddress,
                                           uint16_t registerOffset,
                                           std::span<const uint16_t> registers);

    auto encode() -> void;

  private:
    static constexpr uint8_t commandCode = WriteMultipleRegistersFunctionCode;
    const uint8_t deviceAddress;
    const uint16_t registerOffset;
    // The values to write are held in the registers span
    std::span<const uint16_t> registers;
};

static constexpr uint8_t ReadFileRecordFunctionCode = 0x14;

static constexpr uint8_t FileRecordReferenceType = 0x06;

// A file holds 10000 records, addressed 0 to 9999.
static constexpr uint16_t maxFileRecordNumber = 0x270F;

// A sub request is reftype(1), file(2), record(2), length(2).
static constexpr size_t fileSubRequestLength = 7;

// Sub requests share the PDU with the request's func(1) and bytecount(1).
static constexpr size_t maxFileSubRequests =
    (Message::maxPDUSize - 2) / fileSubRequestLength;

// A sub response is fieldlen(1), reftype(1), then the record's registers.
static constexpr size_t fileSubResponseHeader = 2;

// The field length covers the reftype and the registers, not itself.
static constexpr size_t fileFieldLengthHeader = fileSubResponseHeader - 1;

/** @brief A group of sequential records in one file, read as one sub
 *  request.
 *
 *  @a data is sized to the record length to read, and holds what was read
 *  once the response is decoded. */
struct FileRecord
{
    uint16_t fileNumber = 0;
    uint16_t recordNumber = 0;
    std::span<uint16_t> data;
};

/** @brief Read groups of records from a device's files.
 *
 *  Groups need not be contiguous, and may be in different files, but the
 *  records within a group are sequential. Every group shares one response,
 *  so reading more groups does not read more in total. */
class ReadFileRecordRequest : public Message
{
  public:
    ReadFileRecordRequest() = delete;
    ReadFileRecordRequest(const ReadFileRecordRequest&) = delete;
    ReadFileRecordRequest& operator=(const ReadFileRecordRequest&) = delete;
    ReadFileRecordRequest(ReadFileRecordRequest&&) = delete;
    ReadFileRecordRequest& operator=(ReadFileRecordRequest&&) = delete;

    explicit ReadFileRecordRequest(uint8_t deviceAddress,
                                   std::span<const FileRecord> records);

    auto encode() -> void;

  private:
    static constexpr uint8_t commandCode = ReadFileRecordFunctionCode;
    const uint8_t deviceAddress;
    std::span<const FileRecord> records;
};

class Response : public Message
{
  public:
    auto decode() -> void;
};

class ReadHoldingRegistersResponse : public Response
{
  public:
    ReadHoldingRegistersResponse() = delete;
    ReadHoldingRegistersResponse(const ReadHoldingRegistersResponse&) = delete;
    ReadHoldingRegistersResponse& operator=(
        const ReadHoldingRegistersResponse&) = delete;
    ReadHoldingRegistersResponse(ReadHoldingRegistersResponse&&) = delete;
    ReadHoldingRegistersResponse& operator=(ReadHoldingRegistersResponse&&) =
        delete;

    explicit ReadHoldingRegistersResponse(uint8_t deviceAddress,
                                          std::span<uint16_t> registers);

    auto decode() -> void;

  private:
    static constexpr uint8_t expectedCommandCode =
        ReadHoldingRegistersFunctionCode;
    const uint8_t expectedDeviceAddress;
    // The returned response is stored in the registers span
    std::span<uint16_t> registers;
};

class WriteMultipleRegistersResponse : public Response
{
  public:
    WriteMultipleRegistersResponse() = delete;
    WriteMultipleRegistersResponse(const WriteMultipleRegistersResponse&) =
        delete;
    WriteMultipleRegistersResponse& operator=(
        const WriteMultipleRegistersResponse&) = delete;
    WriteMultipleRegistersResponse(WriteMultipleRegistersResponse&&) = delete;
    WriteMultipleRegistersResponse& operator=(
        WriteMultipleRegistersResponse&&) = delete;

    explicit WriteMultipleRegistersResponse(
        uint8_t deviceAddress, uint16_t registerOffset, uint16_t registerCount);

    auto decode() -> void;

  private:
    static constexpr uint8_t expectedCommandCode =
        WriteMultipleRegistersFunctionCode;
    const uint8_t expectedDeviceAddress;
    const uint16_t expectedRegisterOffset;
    const uint16_t expectedRegisterCount;
};

class ReadFileRecordResponse : public Response
{
  public:
    ReadFileRecordResponse() = delete;
    ReadFileRecordResponse(const ReadFileRecordResponse&) = delete;
    ReadFileRecordResponse& operator=(const ReadFileRecordResponse&) = delete;
    ReadFileRecordResponse(ReadFileRecordResponse&&) = delete;
    ReadFileRecordResponse& operator=(ReadFileRecordResponse&&) = delete;

    explicit ReadFileRecordResponse(uint8_t deviceAddress,
                                    std::span<FileRecord> records);

    auto decode() -> void;

  private:
    static constexpr uint8_t expectedCommandCode = ReadFileRecordFunctionCode;
    const uint8_t expectedDeviceAddress;
    // The returned data is stored in each record's span
    std::span<FileRecord> records;
};

} // namespace phosphor::modbus::rtu
