#include "device_config.hpp"

#include <algorithm>
#include <ctime>
#include <stdexcept>

namespace phosphor::modbus::rtu::device
{

DeviceConfig::DeviceConfig(const config::Config& config, PortIntf& serialPort) :
    config(config), serialPort(serialPort)
{
    for (const auto& reg : config.profile.configRegisters)
    {
        if (reg.period.has_value())
        {
            schedule.push_back(
                {.reg = reg,
                 .interval = std::chrono::seconds(reg.period.value()),
                 .nextWriteTime = {}});
        }
    }
}

auto DeviceConfig::produceValue(ProfileIntf::ConfigType type)
    -> std::vector<uint16_t>
{
    switch (type)
    {
        case ProfileIntf::ConfigType::unixTime:
        {
            auto now = static_cast<uint32_t>(std::time(nullptr));
            return {static_cast<uint16_t>(now >> 16),
                    static_cast<uint16_t>(now & 0xFFFF)};
        }
        case ProfileIntf::ConfigType::unknown:
            throw std::invalid_argument("Unknown config type");
    }
    throw std::invalid_argument("Unknown config type");
}

auto DeviceConfig::writeRegister(const ProfileIntf::ConfigRegister& reg)
    -> sdbusplus::async::task<bool>
{
    auto values = produceValue(reg.type);
    co_return co_await serialPort.writeMultipleRegisters(
        config.address, reg.offset, config.profile.baudRate,
        config.profile.parity, values);
}

auto DeviceConfig::writeInitial() -> sdbusplus::async::task<void>
{
    for (const auto& reg : config.profile.configRegisters)
    {
        if (!reg.period.has_value())
        {
            co_await writeRegister(reg);
        }
    }
}

auto DeviceConfig::writePeriodic()
    -> sdbusplus::async::task<std::chrono::steady_clock::time_point>
{
    auto earliest = std::chrono::steady_clock::time_point::max();
    for (auto& entry : schedule)
    {
        auto now = std::chrono::steady_clock::now();
        if (now < entry.nextWriteTime)
        {
            earliest = std::min(earliest, entry.nextWriteTime);
            continue;
        }
        co_await writeRegister(entry.reg);
        entry.nextWriteTime = std::chrono::steady_clock::now() + entry.interval;
        earliest = std::min(earliest, entry.nextWriteTime);
    }
    co_return earliest;
}

} // namespace phosphor::modbus::rtu::device
