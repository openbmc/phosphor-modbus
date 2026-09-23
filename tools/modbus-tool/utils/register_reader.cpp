#include "utils/register_reader.hpp"

#include "common/register_span.hpp"
#include "inventory/inventory_utils.hpp"
#include "modbus_rtu_config.hpp"
#include "utils/common.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <phosphor-logging/lg2.hpp>

#include <algorithm>
#include <array>
#include <concepts>
#include <functional>
#include <string>
#include <utility>

namespace modbus_tool
{

PHOSPHOR_LOG2_USING;

namespace ModbusIntf = phosphor::modbus::rtu;
namespace InventoryIntf = phosphor::modbus::rtu::inventory;
using phosphor::modbus::buildRegisterSpans;
using phosphor::modbus::RegisterInfo;

namespace
{

/** @brief Turn a profile's registers into the entries the dump reports, one
 *  per register, before anything is read.
 *
 *  Names are prefixed with the device, as the daemon names what it publishes,
 *  so an entry can be found by the name it is known by elsewhere. */
auto toDumpEntries(const std::string& device,
                   const std::vector<ProfileIntf::InventoryRegister>& registers)
    -> std::vector<RegisterDump>
{
    std::vector<RegisterDump> entries;
    for (const auto& reg : registers)
    {
        entries.emplace_back(RegisterDump{
            .name = device + "_" + std::string(inventoryName(reg.type)),
            .offset = reg.offset,
            .size = reg.size,
        });
    }
    return entries;
}

/** @brief A profile register that carries its own name. */
template <typename T>
concept NamedRegister = requires(T reg) {
                            { reg.name } -> std::same_as<std::string&>;
                            { reg.offset } -> std::same_as<uint16_t&>;
                            { reg.size } -> std::same_as<uint8_t&>;
                        };

template <NamedRegister Register>
auto toDumpEntries(const std::string& device,
                   const std::vector<Register>& registers)
    -> std::vector<RegisterDump>
{
    std::vector<RegisterDump> entries;
    for (const auto& reg : registers)
    {
        entries.emplace_back(RegisterDump{
            .name = device + "_" + reg.name,
            .offset = reg.offset,
            .size = reg.size,
        });
    }
    return entries;
}

auto toDumpEntries(const std::string& device,
                   const std::vector<ProfileIntf::StatusRegister>& registers)
    -> std::vector<RegisterDump>
{
    std::vector<RegisterDump> entries;
    for (const auto& reg : registers)
    {
        RegisterDump dump{
            .name = device + "_" + reg.name,
            .offset = reg.offset,
            // Status registers are always a single word.
            .size = 1,
        };
        for (const auto& bit : reg.bits)
        {
            dump.bits.emplace_back(BitDump{
                .position = bit.bitPosition,
                .name = device + "_" + bit.name,
                .type = std::string(statusTypeName(bit.type)),
            });
        }
        entries.emplace_back(std::move(dump));
    }
    return entries;
}

auto toDumpEntries(const std::string& device,
                   const std::vector<ProfileIntf::ConfigRegister>& registers)
    -> std::vector<RegisterDump>
{
    std::vector<RegisterDump> entries;
    for (const auto& reg : registers)
    {
        // Only Init config registers carry a name; the rest are named by type.
        entries.emplace_back(RegisterDump{
            .name = device + "_" +
                    (reg.name == "unknown"
                         ? std::string(configTypeName(reg.type))
                         : reg.name),
            .offset = reg.offset,
            .size = reg.size,
        });
    }
    return entries;
}

/** @brief Fill in whether each modelled bit is set in the word read. */
auto applyBits(RegisterDump& dump) -> void
{
    if (dump.raw.empty())
    {
        return;
    }
    for (auto& bit : dump.bits)
    {
        bit.asserted = ((dump.raw.front() >> bit.position) & 1U) != 0;
    }
}

/** @brief Success only when everything the profile declares was read.
 *
 *  A blackbox that was not asked for is empty, and so reads as complete. */
auto resultFor(const RegisterSet& registers,
               const std::vector<SectionDump>& blackbox) -> Result
{
    const auto groups = {
        std::cref(registers.inventory), std::cref(registers.firmware),
        std::cref(registers.sensor),    std::cref(registers.status),
        std::cref(registers.metric),    std::cref(registers.config)};

    auto allRead = [](const auto& group) {
        return std::ranges::all_of(group.get(), [](const auto& reg) {
            return reg.read;
        });
    };
    auto sectionsRead = std::ranges::all_of(blackbox, [](const auto& section) {
        return section.read;
    });

    return std::ranges::all_of(groups, allRead) && sectionsRead
               ? Result::success
               : Result::partial;
}

} // namespace

RegisterReader::RegisterReader(
    sdbusplus::async::context& ctx,
    const PortIntf::config::PortFactoryConfig& portConfig,
    const std::string& devicePath) : portConfig(portConfig)
{
    fd = open(devicePath.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        error("Failed to open {PATH}", "PATH", devicePath);
        return;
    }

    try
    {
        modbus = std::make_unique<ModbusIntf::Modbus>(
            ctx, fd, portConfig.baudRate, portConfig.rtsDelay,
            portConfig.timeout);
    }
    catch (const std::exception& e)
    {
        error("Failed to open {PATH}: {ERROR}", "PATH", devicePath, "ERROR", e);
        close(fd);
        fd = -1;
    }
}

RegisterReader::~RegisterReader()
{
    modbus.reset();
    if (fd >= 0)
    {
        close(fd);
    }
}

auto RegisterReader::readProbe(const ConfigIntf::Config& config, bool& matched)
    -> sdbusplus::async::task<std::vector<uint16_t>>
{
    const auto& probe = config.profile.probeRegister;
    std::vector<uint16_t> registers(probe.size);

    if (!modbus->setProperties(portConfig.baudRate, config.profile.parity) ||
        !co_await modbus->readHoldingRegisters(config.address, probe.offset,
                                               registers))
    {
        co_return std::vector<uint16_t>{};
    }

    matched = InventoryIntf::matchesProbeValue(registers, probe);
    co_return registers;
}

auto RegisterReader::readGroup(const ConfigIntf::Config& config,
                               const std::vector<RegisterDump>& entries)
    -> sdbusplus::async::task<std::vector<RegisterDump>>
{
    auto group = entries;
    if (group.empty())
    {
        co_return group;
    }

    std::vector<RegisterInfo> infos;
    infos.reserve(group.size());
    for (const auto& reg : group)
    {
        infos.emplace_back(RegisterInfo{
            .offset = reg.offset, .size = static_cast<uint8_t>(reg.size)});
    }

    for (const auto& span :
         buildRegisterSpans(infos, ModbusIntf::maxRegisterSpanLength))
    {
        std::vector<uint16_t> buffer(span.totalSize);
        if (!co_await modbus->readHoldingRegisters(config.address,
                                                   span.startOffset, buffer))
        {
            continue;
        }

        for (auto index : span.registerIndices)
        {
            auto& reg = group[index];
            auto start = reg.offset - span.startOffset;
            reg.raw.assign(buffer.begin() + start,
                           buffer.begin() + start + reg.size);
            reg.read = true;
            applyBits(reg);
        }
    }

    co_return group;
}

auto RegisterReader::readFileSection(const ConfigIntf::Config& config,
                                     uint16_t section, uint16_t length)
    -> sdbusplus::async::task<SectionDump>
{
    SectionDump dump{.section = section};
    dump.raw.reserve(length);

    // A section is longer than one response holds, so walk it a response at
    // a time.
    for (uint16_t record = 0; record < length;)
    {
        auto count = static_cast<uint16_t>(
            std::min<size_t>(length - record, ModbusIntf::maxFileRecordLength));
        std::vector<uint16_t> data(count);
        std::array<ModbusIntf::FileRecord, 1> records{
            {{section, record, data}}};

        if (!co_await modbus->readFileRecord(config.address, records))
        {
            co_return dump;
        }

        dump.raw.insert(dump.raw.end(), data.begin(), data.end());
        record = static_cast<uint16_t>(record + count);
    }

    dump.read = true;
    co_return dump;
}

auto RegisterReader::readBlackbox(const ConfigIntf::Config& config,
                                  const ProfileIntf::Blackbox& blackbox)
    -> sdbusplus::async::task<std::vector<SectionDump>>
{
    std::vector<SectionDump> sections;

    if (blackbox.type != ProfileIntf::BlackboxType::fileRecord)
    {
        error("Unsupported blackbox type for {NAME}", "NAME", config.name);
        co_return sections;
    }

    for (auto section : blackbox.sections)
    {
        sections.emplace_back(
            co_await readFileSection(config, section, blackbox.length));
    }

    co_return sections;
}

auto RegisterReader::read(const ConfigIntf::Config& config, bool withBlackbox)
    -> sdbusplus::async::task<DeviceDump>
{
    DeviceDump dump{
        .name = config.name,
        .type = config.type,
        .address = config.address,
        .serialPort = config.serialPort,
    };

    bool matched = false;
    auto probe = co_await readProbe(config, matched);
    if (probe.empty())
    {
        dump.result = Result::failure;
        dump.reason = "No response";
        co_return dump;
    }

    // The probe register is also an inventory register, so report what the
    // device actually returned even when it is not this variant.
    dump.registers.inventory =
        toDumpEntries(config.name, config.profile.inventoryRegisters);
    for (auto& reg : dump.registers.inventory)
    {
        if (reg.offset == config.profile.probeRegister.offset)
        {
            reg.raw = probe;
            reg.read = true;
        }
    }

    if (!matched)
    {
        dump.result = Result::failure;
        dump.reason = "Probe value mismatch";
        co_return dump;
    }

    co_await readGroups(config, dump.registers);

    if (withBlackbox && config.profile.blackbox)
    {
        dump.blackbox = co_await readBlackbox(config, *config.profile.blackbox);
    }

    dump.result = resultFor(dump.registers, dump.blackbox);
    co_return dump;
}

auto RegisterReader::readGroups(const ConfigIntf::Config& config,
                                RegisterSet& registers)
    -> sdbusplus::async::task<void>
{
    const auto& profile = config.profile;

    // The inventory group already holds what the probe read.
    registers.inventory = co_await readGroup(config, registers.inventory);
    const auto& device = config.name;

    registers.firmware = co_await readGroup(
        config, toDumpEntries(device, profile.firmwareRegisters));
    registers.sensor = co_await readGroup(
        config, toDumpEntries(device, profile.sensorRegisters));
    registers.status = co_await readGroup(
        config, toDumpEntries(device, profile.statusRegisters));
    registers.metric = co_await readGroup(
        config, toDumpEntries(device, profile.metricRegisters));
    registers.config = co_await readGroup(
        config, toDumpEntries(device, profile.configRegisters));
}

} // namespace modbus_tool
