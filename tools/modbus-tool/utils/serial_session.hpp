#pragma once

#include "utils/entity_manager_lookup.hpp"

#include <sdbusplus/async.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace modbus_tool
{

/** @brief The variant of a device that answered its probe.
 *
 *  A second sourced device carries a configuration per variant and only one
 *  is really present, so a command needs to know which answered before it
 *  can address one. */
struct ProbeOutcome
{
    const ConfigIntf::Config* config = nullptr;
    bool answered = false;
};

/** @brief An open serial port and the Modbus conversation over it.
 *
 *  Opens the serial device itself, so the caller reserves the port first. A
 *  session exists only once the port is open, so every operation on one
 *  reaches the bus. */
class SerialSession
{
  public:
    /** @brief Open the serial device and start a conversation over it.
     *  @throws std::runtime_error if the port cannot be opened. */
    SerialSession(sdbusplus::async::context& ctx,
                  const PortIntf::config::PortFactoryConfig& portConfig,
                  const std::string& devicePath);
    SerialSession(const SerialSession&) = delete;
    SerialSession& operator=(const SerialSession&) = delete;
    SerialSession(SerialSession&&) = delete;
    SerialSession& operator=(SerialSession&&) = delete;
    ~SerialSession();

    /** @brief Apply the port settings a device's profile asks for.
     *  @return False if the port would not take them. */
    auto applyProfile(const ConfigIntf::Config& config) -> bool;

    /** @brief Read the probe register and compare it with the profile.
     *  @return The words read, or empty if the device did not answer. */
    auto probe(const ConfigIntf::Config& config, bool& matched)
        -> sdbusplus::async::task<std::vector<uint16_t>>;

    /** @brief Probe each variant of a device and keep the one that is
     *  present. The port is left with the settings of the last variant
     *  probed, so apply the profile again before addressing the device. */
    auto probeVariants(const DeviceVariants& device)
        -> sdbusplus::async::task<ProbeOutcome>;

    auto readHoldingRegisters(uint8_t deviceAddress, uint16_t registerOffset,
                              std::span<uint16_t> registers)
        -> sdbusplus::async::task<bool>;

    auto readFileRecord(uint8_t deviceAddress,
                        std::span<phosphor::modbus::rtu::FileRecord> records)
        -> sdbusplus::async::task<bool>;

    auto writeSingleRegister(uint8_t deviceAddress, uint16_t registerOffset,
                             uint16_t value) -> sdbusplus::async::task<bool>;

    auto writeMultipleRegisters(uint8_t deviceAddress, uint16_t registerOffset,
                                std::span<const uint16_t> registers)
        -> sdbusplus::async::task<bool>;

  private:
    const PortIntf::config::PortFactoryConfig& portConfig;
    int fd = -1;
    std::unique_ptr<phosphor::modbus::rtu::Modbus> modbus;
};

} // namespace modbus_tool
