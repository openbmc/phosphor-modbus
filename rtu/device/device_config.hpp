#pragma once

#include "base_config.hpp"
#include "port/base_port.hpp"

#include <sdbusplus/async.hpp>

#include <chrono>
#include <cstdint>
#include <vector>

namespace phosphor::modbus::rtu::device
{

namespace ProfileIntf = phosphor::modbus::rtu::profile;
using PortIntf = phosphor::modbus::rtu::port::BasePort;

/** @brief Writes config registers to a device.
 *  - Entries without a Period are written once on bring-up.
 *  - Entries with a Period are written once on bring-up and then rewritten
 *    on that interval. */
class DeviceConfig
{
  public:
    DeviceConfig() = delete;

    DeviceConfig(const config::Config& config, PortIntf& serialPort);

    /** @brief Write the one-shot (non-periodic) config registers once. */
    auto writeInitial() -> sdbusplus::async::task<void>;

    /** @brief Write any periodic config registers that are due. The first call
     *  writes them all, since they start due.
     *  @return The earliest time a periodic write is next due. */
    auto writePeriodic()
        -> sdbusplus::async::task<std::chrono::steady_clock::time_point>;

  private:
    // Schedule entry for a periodic config register write, built at
    // construction so the poll loop can write it on its interval.
    struct ScheduleEntry
    {
        const ProfileIntf::ConfigRegister& reg;
        std::chrono::seconds interval;
        std::chrono::steady_clock::time_point nextWriteTime;
    };

    /** @brief Produce the register values to write for a config type. */
    static auto produceValue(ProfileIntf::ConfigType type)
        -> std::vector<uint16_t>;

    /** @brief Write a single config register to the device. */
    auto writeRegister(const ProfileIntf::ConfigRegister& reg)
        -> sdbusplus::async::task<port::OperationStatus>;

    const config::Config& config;
    PortIntf& serialPort;
    std::vector<ScheduleEntry> schedule;
};

} // namespace phosphor::modbus::rtu::device
