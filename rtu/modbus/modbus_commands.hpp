#pragma once

#include "modbus_message.hpp"

#include <span>
#include <vector>

namespace phosphor::modbus::rtu
{

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

} // namespace phosphor::modbus::rtu
