#include "modbus.hpp"

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

static auto getParitySettings(Parity parity, termios& tty) -> bool
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

Modbus::Modbus(sdbusplus::async::context& ctx, int fd, uint32_t baudRate,
               uint16_t rtsDelay) :
    ctx(ctx), fd(fd), baudRate(baudRate), rtsDelay(rtsDelay),
    fdioInstance(ctx, fd)
{
    termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        throw std::runtime_error("Error getting termios");
    }

    if (cfsetspeed(&tty, baudRateMap.at(baudRate)) != 0)
    {
        throw std::runtime_error("Error setting baud rate");
    }

    if (!getParitySettings(parity, tty))
    {
        throw std::runtime_error("Invalid parity");
    }

    tty.c_cflag |= CS8 | CLOCAL | CREAD;
    // Set non-blocking read behavior
    tty.c_cc[VMIN] = 1;  // Minimum characters to read
    tty.c_cc[VTIME] = 0; // Timeout in deciseconds (0 for no timeout)

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        throw std::runtime_error("Error setting termios");
    }

    info("Modbus created successfully");
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
        if (!getParitySettings(inParity, tty))
        {
            error("Invalid parity");
            return false;
        }
    }

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
        // Flush the input & output buffers for the fd
        tcflush(fd, TCIOFLUSH);
        auto ret = write(fd, request.raw.data(), request.len);
        if ((size_t)ret != request.len)
        {
            error(
                "Failed to send read holding registers for {DEVICE_ADDRESS} with {ERROR}",
                "DEVICE_ADDRESS", deviceAddress, "ERROR", strerror(errno));
            co_return false;
        }

        co_await fdioInstance.next();

        ret = read(fd, response.raw.data(), response.len);
        if (ret < 0)
        {
            error(
                "Failed to read holding registers for {DEVICE_ADDRESS} with {ERROR}",
                "DEVICE_ADDRESS", deviceAddress, "ERROR", strerror(errno));
            co_return false;
        }
        if ((size_t)ret != response.len)
        {
            warning(
                "Response for read holding registers for {DEVICE_ADDRESS} is incomplete, expected {EXPECTED} bytes, got {ACTUAL} bytes",
                "DEVICE_ADDRESS", deviceAddress, "EXPECTED", response.len,
                "ACTUAL", ret);
            co_return false;
        }

        response.decode();

        if (rtsDelay)
        {
            // Asynchronously sleep for rts_delay milliseconds in case bus needs
            // to be idle after each transaction
            co_await sdbusplus::async::sleep_for(
                ctx, std::chrono::milliseconds(rtsDelay));
        }
    }
    catch (std::exception& e)
    {
        error(
            "Overall failed to read holding registers for {DEVICE_ADDRESS} with {ERROR}",
            "DEVICE_ADDRESS", deviceAddress, "ERROR", e);
        co_return false;
    }

    co_return true;
}

} // namespace phosphor::modbus::rtu
