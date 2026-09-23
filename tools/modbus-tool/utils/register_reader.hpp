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
    auto read(const ConfigIntf::Config& config, bool withBlackbox = false)
        -> sdbusplus::async::task<DeviceDump>;

  private:
    /** @brief Read the probe register and compare it with the profile.
     *  @return The words read, or empty if the device did not answer. */
    auto readProbe(const ConfigIntf::Config& config, bool& matched)
        -> sdbusplus::async::task<std::vector<uint16_t>>;

    /** @brief Read every section of the device's blackbox.
     *
     *  A section that cannot be read is reported unread rather than
     *  abandoning the rest, since a blackbox is worth having in part. */
    auto readBlackbox(const ConfigIntf::Config& config,
                      const ProfileIntf::Blackbox& blackbox)
        -> sdbusplus::async::task<std::vector<SectionDump>>;

    /** @brief Read one section with the file record function, in as many
     *  reads as its length needs. */
    auto readFileSection(const ConfigIntf::Config& config, uint16_t section,
                         uint16_t length)
        -> sdbusplus::async::task<SectionDump>;

    /** @brief Load one section into the device's window and read it. */
    auto readMailboxSection(const ConfigIntf::Config& config,
                            const ProfileIntf::Blackbox& blackbox,
                            uint16_t section)
        -> sdbusplus::async::task<SectionDump>;

    /** @brief Wait for the status register to say the window is loaded.
     *  @return False if it never was. */
    auto waitForSection(const ConfigIntf::Config& config,
                        const ProfileIntf::Blackbox& blackbox)
        -> sdbusplus::async::task<bool>;

    /** @brief Read a run of registers, in as many reads as its length needs.
     *  @return The registers, or empty if any read failed. */
    auto readBlock(const ConfigIntf::Config& config, uint16_t offset,
                   uint16_t length)
        -> sdbusplus::async::task<std::vector<uint16_t>>;

    /** @brief Read every group the profile declares, in turn. */
    auto readGroups(const ConfigIntf::Config& config, RegisterSet& registers)
        -> sdbusplus::async::task<void>;

    /** @brief Read a group of registers, merged into spans. */
    auto readGroup(const ConfigIntf::Config& config,
                   const std::vector<RegisterDump>& entries)
        -> sdbusplus::async::task<std::vector<RegisterDump>>;

    sdbusplus::async::context& ctx;
    const PortIntf::config::PortFactoryConfig& portConfig;
    int fd = -1;
    std::unique_ptr<phosphor::modbus::rtu::Modbus> modbus;
};

} // namespace modbus_tool
