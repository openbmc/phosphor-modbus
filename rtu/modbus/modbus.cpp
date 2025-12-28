#include "modbus.hpp"

#include "modbus_rtu_config.h"

#include "modbus_commands.hpp"

#include <termios.h>

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>

namespace phosphor::modbus::rtu
{

PHOSPHOR_LOG2_USING;

const std::unordered_map<int, speed_t> baudRateMap = {
    {0, B0},         {50, B50},        {75, B75},       {110, B110},
    {134, B134},     {150, B150},      {200, B200},     {300, B300},
    {600, B600},     {1200, B1200},    {1800, B1800},   {2400, B2400},
    {4800, B4800},   {9600, B9600},    {19200, B19200}, {38400, B38400},
    {57600, B57600}, {115200, B115200}};

Modbus::Modbus(sdbusplus::async::context& ctx, int fd, uint32_t baudRate,
               uint16_t rtsDelay) :
    ctx(ctx), fd(fd), rtsDelay(rtsDelay),
    fdioInstance(ctx, fd,
                 std::chrono::microseconds(MODBUS_RTU_TIMEOUT_MICROSECONDS))
{
    if (!setProperties(baudRate, Parity::even))
    {
        throw std::runtime_error("Failed to set port properties");
    }

    info("Modbus created successfully");
}

static auto applyParitySettings(Parity parity, termios& tty) -> bool
{
    switch (parity)
    {
        case Parity::none:
            tty.c_cflag &= ~PARENB;
            tty.c_iflag &= ~INPCK;
            break;
        case Parity::odd:
            tty.c_cflag |= PARENB;
            tty.c_cflag |= PARODD;
            tty.c_iflag |= INPCK;
            break;
        case Parity::even:
            tty.c_cflag |= PARENB;
            tty.c_cflag &= ~PARODD;
            tty.c_iflag |= INPCK;
            break;
        default:
            error("Invalid parity");
            return false;
    }

    return true;
}

auto Modbus::setProperties(uint32_t inBaudRate, Parity inParity) -> bool
{
    if (inBaudRate == baudRate && inParity == parity)
    {
        return true;
    }

    termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        error("Error getting termios");
        return false;
    }

    if (inBaudRate != baudRate)
    {
        if (cfsetspeed(&tty, baudRateMap.at(inBaudRate)) != 0)
        {
            error("Error setting baud rate");
            return false;
        }
    }

    if (inParity != parity)
    {
        if (!applyParitySettings(inParity, tty))
        {
            error("Invalid parity");
            return false;
        }
    }

    // TODO: We might not need these again.
    tty.c_cflag |= CS8 | CLOCAL | CREAD;
    // Set non-blocking read behavior
    tty.c_cc[VMIN] = 1;  // Minimum characters to read
    tty.c_cc[VTIME] = 0; // Timeout in deciseconds (0 for no timeout)

    if (tcsetattr(fd, TCSAFLUSH, &tty) != 0)
    {
        error("Error setting termios");
        return false;
    }

    parity = inParity;
    baudRate = inBaudRate;

    debug("Properties set successfully");

    return true;
}

static auto printMessage(uint8_t* data, size_t len) -> void
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');

    for (size_t i = 0; i < len; ++i)
    {
        ss << std::setw(2) << static_cast<int>(data[i]) << " ";
    }

    debug("{MSG}", "MSG", ss.str());
}

auto Modbus::readHoldingRegisters(uint8_t deviceAddress,
                                  uint16_t registerOffset,
                                  std::vector<uint16_t>& registers)
    -> sdbusplus::async::task<bool>
{
    try
    {
        ReadHoldingRegistersRequest request(deviceAddress, registerOffset,
                                            registers.size());
        ReadHoldingRegistersResponse response(deviceAddress, registers);

        request.encode();

        debug(
            "Sending read holding registers request for {REGISTER_OFFSET} {DEVICE_ADDRESS}",
            "REGISTER_OFFSET", registerOffset, "DEVICE_ADDRESS", deviceAddress);

        if (!co_await writeRequest(deviceAddress, request))
        {
            co_return false;
        }

        debug(
            "Waiting for read holding registers response for {REGISTER_OFFSET} {DEVICE_ADDRESS}",
            "REGISTER_OFFSET", registerOffset, "DEVICE_ADDRESS", deviceAddress);

        if (!co_await readResponse(deviceAddress, response,
                                   request.functionCode))
        {
            co_return false;
        }

        response.decode();
    }
    catch (std::exception& e)
    {
        error(
            "Failed to read holding registers for {DEVICE_ADDRESS} with {ERROR}",
            "DEVICE_ADDRESS", deviceAddress, "ERROR", e);
        co_return false;
    }

    co_return true;
}

auto Modbus::writeRequest(uint8_t deviceAddress, Message& request)
    -> sdbusplus::async::task<bool>
{
    printMessage(request.raw.data(), request.len);

    // Flush the input & output buffers for the fd
    tcflush(fd, TCIOFLUSH);
    auto ret = write(fd, request.raw.data(), request.len);
    if ((size_t)ret != request.len)
    {
        error("Failed to send request to device {DEVICE_ADDRESS} with {ERROR}",
              "DEVICE_ADDRESS", deviceAddress, "ERROR", strerror(errno));
        co_return false;
    }

    co_return true;
}

auto Modbus::readResponse(uint8_t deviceAddress, Message& response,
                          uint8_t expectedResponseCode)
    -> sdbusplus::async::task<bool>
{
    int expectedLen = response.len;

    do
    {
        debug("Waiting for response for {DEVICE_ADDRESS} with {EXPECTED} bytes",
              "DEVICE_ADDRESS", deviceAddress, "EXPECTED", expectedLen);
        co_await fdioInstance.next();
        // TODO: Handle FD timeout in case of no response
        auto ret = read(fd, response.raw.data() + response.len - expectedLen,
                        expectedLen);
        if (ret < 0)
        {
            error(
                "Failed to read response for device {DEVICE_ADDRESS} with {ERROR}",
                "DEVICE_ADDRESS", deviceAddress, "ERROR", strerror(errno));
            co_return false;
        }

        debug("Received response for {DEVICE_ADDRESS} with {SIZE}",
              "DEVICE_ADDRESS", deviceAddress, "SIZE", ret);

        printMessage(response.raw.data() + response.len - expectedLen, ret);

        expectedLen -= ret;

        if (expectedLen > 0)
        {
            debug(
                "Read response for device {DEVICE_ADDRESS} with {EXPECTED} bytes remaining",
                "DEVICE_ADDRESS", deviceAddress, "EXPECTED", expectedLen);
        }
        if (ret >= 2 && response.functionCode != expectedResponseCode)
        {
            // Update the length of the expected response to received error
            // message size
            response.len = ret;
            error("Received error response {CODE} for device {DEVICE_ADDRESS}",
                  "CODE", response.raw[1], "DEVICE_ADDRESS", deviceAddress);
            co_return false;
        }
    } while (expectedLen > 0);

    if (rtsDelay)
    {
        // Asynchronously sleep for rts_delay milliseconds in case bus needs
        // to be idle after each transaction
        co_await sdbusplus::async::sleep_for(
            ctx, std::chrono::milliseconds(rtsDelay));
    }

    co_return true;
}

} // namespace phosphor::modbus::rtu
