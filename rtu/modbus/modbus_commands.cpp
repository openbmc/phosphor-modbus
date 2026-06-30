#include "modbus_commands.hpp"

#include "modbus_exception.hpp"

#include <phosphor-logging/lg2.hpp>

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
    // addr(1), func(1), bytecount(1), <2 * count regs>, crc(2)
    len = 5 + (2 * registers.size());
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
    // addr(1), func(1), offset(2), count(2), bytecount(1), <2 * count regs>,
    // crc(2)
    auto byteCount = static_cast<uint8_t>(registers.size() * 2);
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
    verifyValue("Byte Count", byteCount, registers.size() * 2);
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

} // namespace phosphor::modbus::rtu
