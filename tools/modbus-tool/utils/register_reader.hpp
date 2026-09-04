#pragma once

#include "utils/dump_types.hpp"
#include "utils/entity_manager_lookup.hpp"

#include <sdbusplus/async.hpp>

#include <string>
#include <vector>

namespace modbus_tool
{

/** @brief Reads devices on one serial port.
 *
 *  Opens the serial device itself and speaks Modbus over it. The port must
 *  already be reserved; nothing here gates against the daemon. */
class RegisterReader
{
  public:
    RegisterReader(sdbusplus::async::context& ctx,
                   const PortIntf::config::PortFactoryConfig& portConfig,
                   const std::string& devicePath);
    RegisterReader(const RegisterReader&) = delete;
    RegisterReader& operator=(const RegisterReader&) = delete;
    RegisterReader(RegisterReader&&) = delete;
    RegisterReader& operator=(RegisterReader&&) = delete;
    ~RegisterReader();

    /** @brief Whether the port was opened. */
    auto ready() const -> bool
    {
        return modbus != nullptr;
    }

    /** @brief Read every register the profile defines.
     *
     *  The probe register is read first, so a device that is absent costs one
     *  read rather than a timeout on every span. Its result is reused for the
     *  inventory register at the same offset. */
    auto read(const ConfigIntf::Config& config)
        -> sdbusplus::async::task<DeviceDump>;

  private:
    /** @brief Read the probe register and compare it with the profile.
     *  @return The words read, or empty if the device did not answer. */
    auto readProbe(const ConfigIntf::Config& config, bool& matched)
        -> sdbusplus::async::task<std::vector<uint16_t>>;

    /** @brief Read every group the profile declares, in turn. */
    auto readGroups(const ConfigIntf::Config& config, RegisterSet& registers)
        -> sdbusplus::async::task<void>;

    /** @brief Read a group of registers, merged into spans. */
    auto readGroup(const ConfigIntf::Config& config,
                   const std::vector<RegisterDump>& entries)
        -> sdbusplus::async::task<std::vector<RegisterDump>>;

    const PortIntf::config::PortFactoryConfig& portConfig;
    int fd = -1;
    std::unique_ptr<phosphor::modbus::rtu::Modbus> modbus;
};

} // namespace modbus_tool
