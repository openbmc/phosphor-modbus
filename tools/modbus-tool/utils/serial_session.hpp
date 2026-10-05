#pragma once

#include "utils/entity_manager_lookup.hpp"

#include <sdbusplus/async.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace modbus_tool
{

/** @brief An open serial port and the Modbus conversation over it.
 *
 *  Opens the serial device itself, so the port must already be reserved;
 *  nothing here gates against the daemon. */
class SerialSession
{
  public:
    SerialSession(sdbusplus::async::context& ctx,
                  const PortIntf::config::PortFactoryConfig& portConfig,
                  const std::string& devicePath);
    SerialSession(const SerialSession&) = delete;
    SerialSession& operator=(const SerialSession&) = delete;
    SerialSession(SerialSession&&) = delete;
    SerialSession& operator=(SerialSession&&) = delete;
    ~SerialSession();

    /** @brief Whether the port was opened. */
    auto ready() const -> bool
    {
        return modbus != nullptr;
    }

    /** @brief The conversation itself. Only valid once ready(). */
    auto bus() -> phosphor::modbus::rtu::Modbus&
    {
        return *modbus;
    }

    /** @brief Apply the port settings a device's profile asks for.
     *  @return False if the port would not take them. */
    auto applyProfile(const ConfigIntf::Config& config) -> bool;

    /** @brief Read the probe register and compare it with the profile.
     *  @return The words read, or empty if the device did not answer. */
    auto probe(const ConfigIntf::Config& config, bool& matched)
        -> sdbusplus::async::task<std::vector<uint16_t>>;

  private:
    const PortIntf::config::PortFactoryConfig& portConfig;
    int fd = -1;
    std::unique_ptr<phosphor::modbus::rtu::Modbus> modbus;
};

} // namespace modbus_tool
