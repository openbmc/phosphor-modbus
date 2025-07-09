#include "serial_port.hpp"

#include "common/entity_manager_interface.hpp"

#include <fcntl.h>

#include <phosphor-logging/lg2.hpp>
#include <xyz/openbmc_project/Configuration/USBPort/client.hpp>

#include <filesystem>
#include <format>
#include <optional>
#include <regex>

namespace phosphor::modbus::rtu::serial_port
{

PHOSPHOR_LOG2_USING;

namespace config
{

static constexpr std::array<std::pair<std::string_view, PortType>, 1>
    validPortTypes = {{{"USBPort", PortType::usb}}};

static constexpr std::array<std::pair<std::string_view, PortMode>, 2>
    validPortModes = {{{"RS232", PortMode::rs232}, {"RS485", PortMode::rs485}}};

auto getConfig(sdbusplus::async::context& ctx,
               sdbusplus::message::object_path objectPath)
    -> sdbusplus::async::task<std::optional<Config>>
{
    Config config = {};

    auto properties =
        co_await USBPortConfigIntf(ctx)
            .service(entity_manager::EntityManagerInterface::serviceName)
            .path(objectPath.str)
            .properties();

    config.name = properties.name;

    for (const auto& [typeStr, portType] : config::validPortTypes)
    {
        if (typeStr == properties.type)
        {
            config.portType = portType;
            break;
        }
    }
    if (config.portType == PortType::unknown)
    {
        error("Invalid port type {PORT_TYPE} for {NAME}", "PORT_TYPE",
              properties.type, "NAME", properties.name);
        co_return std::nullopt;
    }

    for (const auto& [modeStr, portMode] : config::validPortModes)
    {
        if (modeStr == properties.mode)
        {
            config.portMode = portMode;
            break;
        }
    }
    if (config.portMode == PortMode::unknown)
    {
        error("Invalid port mode {PORT_MODE} for {NAME}", "PORT_MODE",
              properties.mode, "NAME", properties.name);
        co_return std::nullopt;
    }

    config.address = properties.device_address;
    config.port = properties.port;
    config.interface = properties.device_interface;
    config.baudRate = properties.baud_rate;
    config.rtsDelay = properties.rts_delay;

    debug(
        "Serial port config: {NAME} {PORT_TYPE} {PORT_MODE} {ADDRESS} {PORT} {INTERFACE} {BAUD_RATE} {RTS_DELAY}",
        "NAME", config.name, "PORT_TYPE", config.portType, "PORT_MODE",
        config.portMode, "ADDRESS", config.address, "PORT", config.port,
        "INTERFACE", config.interface, "BAUD_RATE", config.baudRate,
        "RTS_DELAY", config.rtsDelay);
    co_return config;
}

} // namespace config

static auto getDevicePath(const config::Config& config)
    -> std::optional<std::string>
{
    namespace fs = std::filesystem;
    std::regex pattern(
        std::format("platform-{}\\.usb-usb.*{}-port{}", config.address,
                    config.interface, config.port));
    fs::path searchDir = "/dev/serial/by-path/";

    for (const auto& entry : fs::recursive_directory_iterator(searchDir))
    {
        if (entry.is_symlink())
        {
            auto filePath = entry.path();
            if (std::regex_search(filePath.filename().string(), pattern))
            {
                return ("/dev/" +
                        fs::read_symlink(filePath).filename().string());
            }
        }
    }

    return std::nullopt;
}

SerialPort::SerialPort(sdbusplus::async::context& ctx, config::Config& config) :
    config(config)
{
    auto res = getDevicePath(config);
    if (!res.has_value())
    {
        throw std::runtime_error("Failed to get device path");
    }
    auto devicePath = res.value();

    fd = open(devicePath.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        throw("Failed to open serial port " + devicePath +
              " with error: " + strerror(errno));
    }

    modbus =
        std::make_unique<ModbusIntf>(ctx, fd, config.baudRate, config.rtsDelay);
    if (!modbus)
    {
        throw std::runtime_error("Failed to create Modbus interface");
    }

    info("Serial port {NAME} created successfully", "NAME", config.name);
}

auto SerialPort::readHoldingRegisters(
    uint8_t deviceAddress, uint16_t registerOffset, uint32_t baudRate,
    Parity parity, std::vector<uint16_t>& registers)
    -> sdbusplus::async::task<bool>
{
    if (!modbus->setProperties(baudRate, parity))
    {
        error("Failed to set serial port properties");
        co_return false;
    }

    // TODO: Acquire asyncscoped mutex

    debug("Reading holding registers from device {ADDRESS} at offset {OFFSET}",
          "ADDRESS", deviceAddress, "OFFSET", registerOffset);

    auto ret = co_await modbus->readHoldingRegisters(deviceAddress,
                                                     registerOffset, registers);
    if (!ret)
    {
        error(
            "Failed to read holding registers from device {ADDRESS} at offset "
            "{OFFSET}",
            "ADDRESS", deviceAddress, "OFFSET", registerOffset);
    }

    co_return ret;
}

} // namespace phosphor::modbus::rtu::serial_port
