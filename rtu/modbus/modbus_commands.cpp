#include "modbus_commands.hpp"

#include "modbus_exception.hpp"

#include <phosphor-logging/lg2.hpp>

#include <stdexcept>

namespace phosphor::modbus::rtu
{

ReadHoldingRegistersRequest::ReadHoldingRegistersRequest(
    uint8_t deviceAddress, uint16_t registerOffset, uint16_t registerCount) :
    deviceAddress(deviceAddress), registerOffset(registerOffset),
    registerCount(registerCount)
{}

auto ReadHoldingRegistersRequest::encode() -> void
{
    *this << deviceAddress << commandCode << registerOffset << registerCount;
    appendCRC();
}

ReadHoldingRegistersResponse::ReadHoldingRegistersResponse(
    uint8_t deviceAddress, std::span<uint16_t> registers) :
    expectedDeviceAddress(deviceAddress), registers(registers)
{
    if (registers.empty())
    {
        throw std::underflow_error("Response registers are empty");
    }
    // addr(1), func(1), bytecount(1), <registerSize * count regs>, crc(2)
    len = 5 + (registerSize * registers.size());
}

WriteSingleRegisterRequest::WriteSingleRegisterRequest(
    uint8_t deviceAddress, uint16_t registerOffset, uint16_t value) :
    deviceAddress(deviceAddress), registerOffset(registerOffset), value(value)
{}

auto WriteSingleRegisterRequest::encode() -> void
{
    // addr(1), func(1), offset(2), value(2), crc(2)
    *this << deviceAddress << commandCode << registerOffset << value;
    appendCRC();
}

WriteSingleRegisterResponse::WriteSingleRegisterResponse(
    uint8_t deviceAddress, uint16_t registerOffset, uint16_t value) :
    expectedDeviceAddress(deviceAddress),
    expectedRegisterOffset(registerOffset), expectedValue(value)
{
    // The response echoes the request, so it is the same size.
    len = 8;
}

WriteMultipleRegistersRequest::WriteMultipleRegistersRequest(
    uint8_t deviceAddress, uint16_t registerOffset,
    std::span<const uint16_t> registers) :
    deviceAddress(deviceAddress), registerOffset(registerOffset),
    registers(registers)
{
    if (registers.empty())
    {
        throw std::underflow_error("No registers to write");
    }
}

auto WriteMultipleRegistersRequest::encode() -> void
{
    // addr(1), func(1), offset(2), count(2), bytecount(1),
    // <registerSize * count regs>, crc(2)
    auto byteCount = static_cast<uint8_t>(registerSize * registers.size());
    *this << deviceAddress << commandCode << registerOffset
          << static_cast<uint16_t>(registers.size()) << byteCount << registers;
    appendCRC();
}

WriteMultipleRegistersResponse::WriteMultipleRegistersResponse(
    uint8_t deviceAddress, uint16_t registerOffset, uint16_t registerCount) :
    expectedDeviceAddress(deviceAddress),
    expectedRegisterOffset(registerOffset), expectedRegisterCount(registerCount)
{
    // addr(1), func(1), offset(2), count(2), crc(2)
    len = 8;
}

ReadFileRecordRequest::ReadFileRecordRequest(
    uint8_t deviceAddress, std::span<const FileRecord> records) :
    deviceAddress(deviceAddress), records(records)
{
    if (records.empty())
    {
        throw std::underflow_error("No records to read");
    }
    if (records.size() > maxFileSubRequests)
    {
        throw std::overflow_error("Too many sub requests for one request");
    }
    for (const auto& record : records)
    {
        if (record.data.empty())
        {
            throw std::underflow_error("A record has no length");
        }
        if (record.recordNumber > maxFileRecordNumber)
        {
            throw std::out_of_range("Record number past the end of a file");
        }
    }
}

auto ReadFileRecordRequest::encode() -> void
{
    // addr(1), func(1), bytecount(1), <7 * records>, crc(2)
    auto byteCount =
        static_cast<uint8_t>(records.size() * fileSubRequestLength);
    *this << deviceAddress << commandCode << byteCount;
    for (const auto& record : records)
    {
        *this << FileRecordReferenceType << record.fileNumber
              << record.recordNumber
              << static_cast<uint16_t>(record.data.size());
    }
    appendCRC();
}

ReadFileRecordResponse::ReadFileRecordResponse(uint8_t deviceAddress,
                                               std::span<FileRecord> records) :
    expectedDeviceAddress(deviceAddress), records(records)
{
    if (records.empty())
    {
        throw std::underflow_error("Response records are empty");
    }
    // addr(1), func(1), datalen(1), <a sub response per record>, crc(2)
    len = 5;
    for (const auto& record : records)
    {
        len += fileSubResponseHeader + (registerSize * record.data.size());
    }
    if (len > maxADUSize)
    {
        throw std::overflow_error("Records do not fit one response");
    }
}

auto Response::decode() -> void
{
    validate();

    // Error response is structured as:
    // addr(1), errorFunctionCode(1), exceptionCode(1)
    // Where errorFunctionCode is the response function with
    // MSB set to 1, hence mask of 0x80.
    bool isError = (len == 3 && (functionCode & 0x80) != 0);
    if (isError)
    {
        throw ModbusException(raw[2]);
    }
}

auto ReadHoldingRegistersResponse::decode() -> void
{
    Response::decode();
    uint8_t byteCount, responseCode, deviceAddress;
    *this >> registers >> byteCount >> responseCode >> deviceAddress;
    verifyValue("Device Address", deviceAddress, expectedDeviceAddress);
    verifyValue("Response Function Code", responseCode, expectedCommandCode);
    verifyValue("Byte Count", byteCount, registerSize * registers.size());
}

auto WriteSingleRegisterResponse::decode() -> void
{
    Response::decode();
    uint16_t value, registerOffset;
    uint8_t responseCode, deviceAddress;
    *this >> value >> registerOffset >> responseCode >> deviceAddress;
    verifyValue("Device Address", deviceAddress, expectedDeviceAddress);
    verifyValue("Response Function Code", responseCode, expectedCommandCode);
    verifyValue("Register Offset", registerOffset, expectedRegisterOffset);
    verifyValue("Value", value, expectedValue);
}

auto WriteMultipleRegistersResponse::decode() -> void
{
    Response::decode();
    uint16_t registerCount, registerOffset;
    uint8_t responseCode, deviceAddress;
    *this >> registerCount >> registerOffset >> responseCode >> deviceAddress;
    verifyValue("Device Address", deviceAddress, expectedDeviceAddress);
    verifyValue("Response Function Code", responseCode, expectedCommandCode);
    verifyValue("Register Offset", registerOffset, expectedRegisterOffset);
    verifyValue("Register Count", registerCount, expectedRegisterCount);
}

auto ReadFileRecordResponse::decode() -> void
{
    Response::decode();

    // Records come off the end, so unwind them back to front.
    size_t dataLengthExpected = 0;
    for (auto record = records.rbegin(); record != records.rend(); record++)
    {
        uint8_t referenceType, fieldLength;
        *this >> record->data >> referenceType >> fieldLength;
        verifyValue("Reference Type", referenceType, FileRecordReferenceType);
        verifyValue("Field Length", fieldLength,
                    fileFieldLengthHeader +
                        (registerSize * record->data.size()));
        dataLengthExpected +=
            fileSubResponseHeader + (registerSize * record->data.size());
    }

    uint8_t dataLength, responseCode, deviceAddress;
    *this >> dataLength >> responseCode >> deviceAddress;
    verifyValue("Device Address", deviceAddress, expectedDeviceAddress);
    verifyValue("Response Function Code", responseCode, expectedCommandCode);
    verifyValue("Data Length", dataLength, dataLengthExpected);
    // Anything left is a field the response should not have carried.
    verifyValue("Unread Length", len, 0);
}

} // namespace phosphor::modbus::rtu
