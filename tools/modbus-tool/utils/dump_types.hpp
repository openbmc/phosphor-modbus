#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace modbus_tool
{

/** @brief How much of a device could be read. */
enum class Result
{
    success, // Every register read.
    partial, // Some registers failed.
    failure, // Nothing was read; reason says why.
};

/** @brief One bit of a status register, as the profile defines it. */
struct BitDump
{
    uint8_t position = 0;
    std::string name{};
    std::string type{};
    bool asserted = false;
};

/** @brief One register, reported as the words the device returned. */
struct RegisterDump
{
    std::string name{};
    uint16_t offset = 0;
    uint16_t size = 0;
    bool read = false;
    std::vector<uint16_t> raw{};
    // Status registers only; empty for every other class.
    std::vector<BitDump> bits{};
};

/** @brief A device's registers, grouped as the profile groups them. */
struct RegisterSet
{
    std::vector<RegisterDump> inventory{};
    std::vector<RegisterDump> firmware{};
    std::vector<RegisterDump> sensor{};
    std::vector<RegisterDump> status{};
    std::vector<RegisterDump> metric{};
    std::vector<RegisterDump> config{};
};

/** @brief One section of a device's blackbox. */
struct SectionDump
{
    uint16_t section = 0;
    bool read = false;
    std::vector<uint16_t> raw{};
};

struct DeviceDump
{
    std::string name{};
    std::string type{};
    uint8_t address = 0;
    std::string serialPort{};
    Result result = Result::failure;
    // Only set when result is failure.
    std::string reason{};
    RegisterSet registers{};
    // Only read when asked for.
    std::vector<SectionDump> blackbox{};
};

struct Dump
{
    std::vector<DeviceDump> devices{};
};

} // namespace modbus_tool
