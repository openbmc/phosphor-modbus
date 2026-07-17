#include "modbus.hpp"

#include "modbus_commands.hpp"
#include "modbus_rtu_config.hpp"

#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

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
               uint16_t rtsDelay, std::chrono::microseconds timeout) :
    ctx(ctx), fd(fd), rtsDelay(rtsDelay), fdioInstance(ctx, fd, timeout)
{
    if (!setProperties(baudRate, Parity::even))
    {
        throw std::runtime_error("Failed to set port properties");
    }

    setRS485Config();

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

    termios tty{};
    if (cfsetspeed(&tty, baudRateMap.at(inBaudRate)) != 0)
    {
        error("Error setting baud rate");
        return false;
    }

    if (!applyParitySettings(inParity, tty))
    {
        error("Invalid parity");
        return false;
    }

    tty.c_cflag |= CS8 | CLOCAL | CREAD;
    // In non-canonical mode, block until at least VMIN bytes are available.
    // See termios(3).
    tty.c_cc[VMIN] = 1;
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

auto Modbus::setRS485Config() -> void
{
    /*
     * RS485 ioctl is required by some drivers (e.g. xr_serial) but unsupported
     * by others (e.g. ftdi_sio), so failures are non-fatal.
     */
    struct serial_rs485 rs485Conf{};

    rs485Conf.flags = SER_RS485_ENABLED | SER_RS485_RTS_AFTER_SEND |
                      SER_RS485_RX_DURING_TX;

    if (ioctl(fd, TIOCSRS485, &rs485Conf) < 0)
    {
        const int err = errno;

        if (err == ENOTTY)
        {
            info("Serial driver does not support TIOCSRS485; "
                 "continuing without kernel RS485 configuration");
            return;
        }

        warning("Failed to configure RS485 mode: {ERR}", "ERR", strerror(err));
    }
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
                                  std::span<uint16_t> registers)
    -> sdbusplus::async::task<bool>
{
    for (uint8_t attempt = 0; attempt <= modbusRTURetries; ++attempt)
    {
        try
        {
            ReadHoldingRegistersRequest request(deviceAddress, registerOffset,
                                                registers.size());
            ReadHoldingRegistersResponse response(deviceAddress, registers);

            request.encode();

            debug(
                "Sending read holding registers request for {REGISTER_OFFSET} {DEVICE_ADDRESS}",
                "REGISTER_OFFSET", lg2::hex, registerOffset, "DEVICE_ADDRESS",
                lg2::hex, deviceAddress);

            if (!co_await writeRequest(deviceAddress, request))
            {
                continue;
            }

            debug(
                "Waiting for read holding registers response for {REGISTER_OFFSET} {DEVICE_ADDRESS}",
                "REGISTER_OFFSET", lg2::hex, registerOffset, "DEVICE_ADDRESS",
                lg2::hex, deviceAddress);

            if (!co_await readResponse(deviceAddress, response,
                                       request.functionCode))
            {
                continue;
            }

            response.decode();
            co_return true;
        }
        catch (std::exception& e)
        {
            if (attempt == modbusRTURetries)
            {
                error(
                    "Failed to read holding registers for {DEVICE_ADDRESS} with {ERROR}",
                    "DEVICE_ADDRESS", lg2::hex, deviceAddress, "ERROR", e);
            }
        }
    }

    co_return false;
}

auto Modbus::writeMultipleRegisters(uint8_t deviceAddress,
                                    uint16_t registerOffset,
                                    std::span<const uint16_t> registers)
    -> sdbusplus::async::task<bool>
{
    for (uint8_t attempt = 0; attempt <= modbusRTURetries; ++attempt)
    {
        try
        {
            WriteMultipleRegistersRequest request(deviceAddress, registerOffset,
                                                  registers);
            WriteMultipleRegistersResponse response(
                deviceAddress, registerOffset, registers.size());

            request.encode();

            debug(
                "Sending write multiple registers request for {REGISTER_OFFSET} {DEVICE_ADDRESS}",
                "REGISTER_OFFSET", lg2::hex, registerOffset, "DEVICE_ADDRESS",
                lg2::hex, deviceAddress);

            if (!co_await writeRequest(deviceAddress, request))
            {
                continue;
            }

            debug(
                "Waiting for write multiple registers response for {REGISTER_OFFSET} {DEVICE_ADDRESS}",
                "REGISTER_OFFSET", lg2::hex, registerOffset, "DEVICE_ADDRESS",
                lg2::hex, deviceAddress);

            if (!co_await readResponse(deviceAddress, response,
                                       request.functionCode))
            {
                continue;
            }

            response.decode();
            co_return true;
        }
        catch (std::exception& e)
        {
            if (attempt == modbusRTURetries)
            {
                error(
                    "Failed to write multiple registers for {DEVICE_ADDRESS} with {ERROR}",
                    "DEVICE_ADDRESS", lg2::hex, deviceAddress, "ERROR", e);
            }
        }
    }

    co_return false;
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
              "DEVICE_ADDRESS", lg2::hex, deviceAddress, "ERROR",
              strerror(errno));
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
              "DEVICE_ADDRESS", lg2::hex, deviceAddress, "EXPECTED",
              expectedLen);
        co_await fdioInstance.next();
        auto ret = read(fd, response.raw.data() + response.len - expectedLen,
                        expectedLen);
        if (ret < 0)
        {
            error(
                "Failed to read response for device {DEVICE_ADDRESS} with {ERROR}",
                "DEVICE_ADDRESS", lg2::hex, deviceAddress, "ERROR",
                strerror(errno));
            co_return false;
        }

        debug("Received response for {DEVICE_ADDRESS} with {SIZE}",
              "DEVICE_ADDRESS", lg2::hex, deviceAddress, "SIZE", ret);

        printMessage(response.raw.data() + response.len - expectedLen, ret);

        expectedLen -= ret;

        if (expectedLen > 0)
        {
            debug(
                "Read response for device {DEVICE_ADDRESS} with {EXPECTED} bytes remaining",
                "DEVICE_ADDRESS", lg2::hex, deviceAddress, "EXPECTED",
                expectedLen);
        }
        if (ret >= 2 && response.functionCode != expectedResponseCode)
        {
            // Update the length of the expected response to received error
            // message size
            response.len = ret;
            error("Received error response {CODE} for device {DEVICE_ADDRESS}",
                  "CODE", response.raw[1], "DEVICE_ADDRESS", lg2::hex,
                  deviceAddress);
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
