#pragma once

#include <format>
#include <stdexcept>
#include <string>

namespace phosphor::modbus::rtu
{

enum class ModbusExceptionCode
{
    // The Modbus function code in the request is not supported or is an invalid
    // action for the server.
    illegalFunctionCode = 1,

    // The requested data address is not valid or authorized for the server.
    illegalDataAddress = 2,

    // The value provided in the request is not an allowable value for the
    // server, or the data field is structured incorrectly.
    illegalDataValue = 3,

    // The server encountered an internal error and cannot perform the requested
    // operation.
    slaveDeviceFailure = 4,

    //  The server has accepted the request but needs more time to process it.
    acknowledge = 5,

    // The server is currently busy and cannot respond to the request.
    slaveDeviceBusy = 6,

    // The server cannot perform the program function received in the query.
    // This code is returned for an unsuccessful programming request using
    // function code 13 or 14 decimal. The client should request diagnostic or
    // error information from the server.
    negativeAcknowledge = 7,

    // The server attempted to read extended memory, but detected a parity error
    // in the memory. The client can retry the request, but service may be
    // required on the server device.
    memoryParityError = 8,
    unknownError = 255,
};

class ModbusException : public std::runtime_error
{
  public:
    const ModbusExceptionCode code;

    explicit ModbusException(uint8_t code, const std::string& message = "") :
        std::runtime_error(std::format(
            "{} ({})", toString(static_cast<ModbusExceptionCode>(code)),
            message)),
        code(static_cast<ModbusExceptionCode>(code))
    {}

    static auto toString(ModbusExceptionCode code) -> std::string
    {
        switch (code)
        {
            case ModbusExceptionCode::illegalFunctionCode:
                return "Illegal Function Code";
            case ModbusExceptionCode::illegalDataAddress:
                return "Illegal Data Address";
            case ModbusExceptionCode::illegalDataValue:
                return "Illegal Data Value";
            case ModbusExceptionCode::slaveDeviceFailure:
                return "Slave Device Failure";
            case ModbusExceptionCode::acknowledge:
                return "Acknowledge";
            case ModbusExceptionCode::slaveDeviceBusy:
                return "Slave Device Busy";
            case ModbusExceptionCode::negativeAcknowledge:
                return "Negative Acknowledge";
            case ModbusExceptionCode::memoryParityError:
                return "Memory Parity Error";
            default:
                return "Unknown Modbus Error";
        }
    }
};

class ModbusCRCException : public std::runtime_error
{
  public:
    explicit ModbusCRCException(uint16_t expectedCRC, uint16_t crc) :
        std::runtime_error(
            "CRC mismatch, expected: " + std::to_string(expectedCRC) +
            " received: " + std::to_string(crc))
    {}
};

class ModbusBadResponseException : public std::runtime_error
{
  public:
    explicit ModbusBadResponseException(const std::string& fieldName,
                                        uint32_t expectedValue,
                                        uint32_t currentValue) :
        std::runtime_error(
            "Value mismatch for " + fieldName +
            ", expected value: " + std::to_string(expectedValue) +
            ", current value: " + std::to_string(currentValue))
    {}
};

} // namespace phosphor::modbus::rtu
