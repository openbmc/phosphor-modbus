#pragma once

#include "modbus_commands.hpp"
#include "modbus_rtu_config.hpp"

#include <sdbusplus/async.hpp>

#include <cstdint>
#include <span>

namespace phosphor::modbus::rtu
{

enum class Parity
{
    odd,
    even,
    none,
    unknown
};

class Modbus
{
  public:
    explicit Modbus(sdbusplus::async::context& ctx, int fd, uint32_t baudRate,
                    uint16_t rtsDelay, std::chrono::microseconds timeout);

    auto setProperties(uint32_t inBaudRate, Parity inParity) -> bool;

    auto readHoldingRegisters(uint8_t deviceAddress, uint16_t registerOffset,
                              std::span<uint16_t> registers,
                              uint8_t retries = modbusRTURetries)
        -> sdbusplus::async::task<bool>;

    /** @brief Read records from a device's files, in one transaction.
     *  @param records  Each with its data sized to the length to read, and
     *                  filled on success.
     *  @return Whether the read succeeded. */
    auto readFileRecord(uint8_t deviceAddress, std::span<FileRecord> records,
                        uint8_t retries = modbusRTURetries)
        -> sdbusplus::async::task<bool>;

    /** @brief Write one register, which the device echoes back. */
    auto writeSingleRegister(uint8_t deviceAddress, uint16_t registerOffset,
                             uint16_t value, uint8_t retries = modbusRTURetries)
        -> sdbusplus::async::task<bool>;

    auto writeMultipleRegisters(uint8_t deviceAddress, uint16_t registerOffset,
                                std::span<const uint16_t> registers,
                                uint8_t retries = modbusRTURetries)
        -> sdbusplus::async::task<bool>;

  private:
    auto writeRequest(uint8_t deviceAddress, Message& request)
        -> sdbusplus::async::task<bool>;

    auto readResponse(uint8_t deviceAddress, Message& response,
                      uint8_t expectedResponseCode)
        -> sdbusplus::async::task<bool>;

    auto setRS485Config() -> void;

    sdbusplus::async::context& ctx;
    int fd;
    uint16_t rtsDelay;
    uint32_t baudRate = 0;
    Parity parity = Parity::unknown;
    sdbusplus::async::fdio fdioInstance;
};

} // namespace phosphor::modbus::rtu
