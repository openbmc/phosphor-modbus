#pragma once

#include "modbus/modbus.hpp"

#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Configuration/USBPort/client.hpp>

#include <concepts>

namespace phosphor::modbus::rtu::port
{

using ModbusIntf = phosphor::modbus::rtu::Modbus;

namespace config
{

enum class PortMode
{
    rs232,
    rs485,
    unknown
};

static constexpr std::array<std::pair<std::string_view, PortMode>, 2>
    validPortModes = {{{"RS232", PortMode::rs232}, {"RS485", PortMode::rs485}}};

struct Config
{
    std::string name = "unknown";
    PortMode portMode = PortMode::unknown;
    uint32_t baudRate = 0;
    uint16_t rtsDelay = 0;
};

template <typename T>
concept HasPropertiesMembers = requires(T properties) {
                                   {
                                       properties.name
                                   } -> std::same_as<std::string&>;
                                   {
                                       properties.mode
                                   } -> std::same_as<std::string&>;
                                   {
                                       properties.baud_rate
                                   } -> std::same_as<uint64_t&>;
                                   {
                                       properties.rts_delay
                                   } -> std::same_as<uint64_t&>;
                               };

template <typename T>
concept HasConfigMembers = requires(T config) {
                               { config.name } -> std::same_as<std::string&>;
                               { config.portMode } -> std::same_as<PortMode&>;
                               { config.baudRate } -> std::same_as<uint32_t&>;
                               { config.rtsDelay } -> std::same_as<uint16_t&>;
                           };

template <HasConfigMembers BaseConfig, HasPropertiesMembers BaseProperties>
auto updateBaseConfig(BaseConfig& config, const BaseProperties& properties)
    -> bool
{
    PHOSPHOR_LOG2_USING;

    config.name = properties.name;
    config.baudRate = static_cast<uint32_t>(properties.baud_rate);
    config.rtsDelay = static_cast<uint16_t>(properties.rts_delay);

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
        return false;
    }

    debug("Base Port config: {NAME} {PORT_MODE} {BAUD_RATE} {RTS_DELAY}",
          "NAME", config.name, "PORT_MODE", config.portMode, "BAUD_RATE",
          config.baudRate, "RTS_DELAY", config.rtsDelay);

    return true;
}

} // namespace config

class BasePort
{
  public:
    explicit BasePort(sdbusplus::async::context& ctx,
                      const config::Config& config,
                      const std::string& devicePath);

    auto readHoldingRegisters(uint8_t deviceAddress, uint16_t registerOffset,
                              uint32_t baudRate, Parity parity,
                              std::vector<uint16_t>& registers)
        -> sdbusplus::async::task<bool>;

  private:
    std::string name;
    int fd = -1;
    std::unique_ptr<ModbusIntf> modbus;
    sdbusplus::async::mutex mutex;
};

} // namespace phosphor::modbus::rtu::port
