#pragma once

#include <sdbusplus/async.hpp>

namespace phosphor::modbus::rtu
{

class Message;

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
                    uint16_t rtsDelay);

    auto setProperties(uint32_t inBaudRate, Parity inParity) -> bool;

    auto readHoldingRegisters(uint8_t deviceAddress, uint16_t registerOffset,
                              std::vector<uint16_t>& registers)
        -> sdbusplus::async::task<bool>;

  private:
    auto writeRequest(uint8_t deviceAddress, Message& request)
        -> sdbusplus::async::task<bool>;

    auto readResponse(uint8_t deviceAddress, Message& response)
        -> sdbusplus::async::task<bool>;

    sdbusplus::async::context& ctx;
    int fd;
    uint32_t baudRate;
    uint16_t rtsDelay;
    Parity parity = Parity::even;
    sdbusplus::async::fdio fdioInstance;
};

} // namespace phosphor::modbus::rtu
