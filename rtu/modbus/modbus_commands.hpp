#pragma once

#include "modbus_message.hpp"

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
                                          std::vector<uint16_t>& registers);

    auto decode() -> void;

  private:
    static constexpr uint8_t expectedCommandCode =
        ReadHoldingRegistersFunctionCode;
    const uint8_t expectedDeviceAddress;
    // The returned response is stored in the registers vector
    std::vector<uint16_t>& registers;
};

} // namespace phosphor::modbus::rtu
