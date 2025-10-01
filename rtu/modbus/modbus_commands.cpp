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
    uint8_t deviceAddress, std::vector<uint16_t>& registers) :
    expectedDeviceAddress(deviceAddress), registers(registers)
{
    if (registers.empty())
    {
        throw std::underflow_error("Response registers are empty");
    }
    // addr(1), func(1), bytecount(1), <2 * count regs>, crc(2)
    len = 5 + (2 * registers.size());
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

} // namespace phosphor::modbus::rtu
