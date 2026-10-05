#include "utils/register_reader.hpp"

#include "common/register_span.hpp"
#include "device/device_utils.hpp"
#include "firmware/firmware_utils.hpp"
#include "inventory/inventory_utils.hpp"
#include "modbus_rtu_config.hpp"
#include "utils/common.hpp"

#include <phosphor-logging/lg2.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <concepts>
#include <functional>
#include <string>
#include <utility>

namespace modbus_tool
{

PHOSPHOR_LOG2_USING;

namespace ModbusIntf = phosphor::modbus::rtu;
namespace DeviceIntf = phosphor::modbus::rtu::device;
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

/** @brief The unit as the dump names it: DegreesC, rather than the D-Bus
 *  name xyz.openbmc_project.Sensor.Value.Unit.DegreesC. */
template <typename Unit>
auto unitName(Unit unit) -> std::string
{
    auto name = convertForMessage(unit);
    return name.substr(name.rfind('.') + 1);
}

/** @brief A number JSON can hold, or no value for one it cannot. */
auto toValue(double number) -> RegisterValue
{
    return std::isfinite(number) ? RegisterValue{number} : RegisterValue{};
}

/** @brief Set the processed value of each register that read successfully.
 *  A register that failed to read gets an empty value, written as null.
 *
 *  Entries are made from the profile's registers in order, so the two line
 *  up. */
template <typename Register, typename Decode>
auto decodeValues(std::vector<RegisterDump>& dumps,
                  const std::vector<Register>& registers, Decode decode) -> void
{
    for (size_t i = 0; i < dumps.size() && i < registers.size(); i++)
    {
        auto& dump = dumps[i];
        dump.value = dump.read ? decode(registers[i], dump.raw)
                               : RegisterValue{};
    }
}

/** @brief A processed string register, without the nulls the device pads it
 *  with, as the probe check drops them too. */
auto toStringValue(std::string value) -> RegisterValue
{
    std::erase(value, '\0');
    return value;
}

/** @brief Fill in the processed value of each register, and the unit of
 *  each sensor and metric. */
auto decodeRegisters(const ProfileIntf::DeviceProfile& profile,
                     RegisterSet& registers) -> void
{
    decodeValues(registers.inventory, profile.inventoryRegisters,
                 [](const auto& reg, const auto& raw) {
                     return toStringValue(
                         InventoryIntf::convertRegisterValue(raw, reg));
                 });
    decodeValues(registers.firmware, profile.firmwareRegisters,
                 [](const auto& reg, const auto& raw) {
                     return toStringValue(
                         DeviceIntf::convertRegisterValue(raw, reg));
                 });

    auto toNumber = [](const auto& reg, const auto& raw) {
        return toValue(DeviceIntf::convertRegisterValue(
            raw, reg.format, reg.isSigned, reg.precision, reg.scale,
            reg.shift));
    };
    decodeValues(registers.sensor, profile.sensorRegisters, toNumber);
    decodeValues(registers.metric, profile.metricRegisters, toNumber);

    decodeValues(registers.config, profile.configRegisters,
                 [](const auto&, const auto& raw) -> RegisterValue {
                     // Wider than 64 bits is not an integer JSON can hold.
                     if (raw.size() > 4)
                     {
                         return {};
                     }
                     uint64_t value = 0;
                     for (auto word : raw)
                     {
                         value = (value << 16) | word;
                     }
                     return value;
                 });

    for (size_t i = 0; i < registers.sensor.size(); i++)
    {
        registers.sensor[i].unit =
            unitName(DeviceIntf::getUnit(profile.sensorRegisters[i].type));
    }
    for (size_t i = 0; i < registers.metric.size(); i++)
    {
        registers.metric[i].unit = unitName(
            DeviceIntf::getMetricUnit(profile.metricRegisters[i].type));
    }
}

// A section is not loaded the instant it is asked for, so give it a moment,
// the way the reference implementation does.
constexpr int sectionReadyRetries = 3;
constexpr auto sectionReadyInterval = std::chrono::seconds(1);

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
    const std::string& devicePath) :
    ctx(ctx), session(ctx, portConfig, devicePath)
{}

RegisterReader::~RegisterReader() = default;

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
        if (!co_await session.bus().readHoldingRegisters(
                config.address, span.startOffset, buffer))
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

        if (!co_await session.bus().readFileRecord(config.address, records))
        {
            co_return dump;
        }

        dump.raw.insert(dump.raw.end(), data.begin(), data.end());
        record = static_cast<uint16_t>(record + count);
    }

    dump.read = true;
    co_return dump;
}

auto RegisterReader::readBlock(const ConfigIntf::Config& config,
                               uint16_t offset, uint16_t length)
    -> sdbusplus::async::task<std::vector<uint16_t>>
{
    std::vector<uint16_t> block;
    block.reserve(length);

    for (uint16_t read = 0; read < length;)
    {
        auto count = static_cast<uint16_t>(
            std::min<size_t>(length - read, ModbusIntf::maxRegisterSpanLength));
        std::vector<uint16_t> part(count);

        if (!co_await session.bus().readHoldingRegisters(
                config.address, static_cast<uint16_t>(offset + read), part))
        {
            co_return std::vector<uint16_t>{};
        }

        block.insert(block.end(), part.begin(), part.end());
        read = static_cast<uint16_t>(read + count);
    }

    co_return block;
}

auto RegisterReader::waitForSection(const ConfigIntf::Config& config,
                                    const ProfileIntf::Blackbox& blackbox)
    -> sdbusplus::async::task<bool>
{
    for (int attempt = 0; attempt < sectionReadyRetries; attempt++)
    {
        // Ask first, and only wait if the section is not loaded yet.
        if (attempt != 0)
        {
            co_await sdbusplus::async::sleep_for(ctx, sectionReadyInterval);
        }

        // Ready once the status reads, and no longer holds the busy value.
        std::array<uint16_t, 1> status{};
        if (co_await session.bus().readHoldingRegisters(
                config.address, blackbox.statusRegister, status) &&
            status[0] != blackbox.busyValue)
        {
            co_return true;
        }
    }

    co_return false;
}

auto RegisterReader::readMailboxSection(const ConfigIntf::Config& config,
                                        const ProfileIntf::Blackbox& blackbox,
                                        uint16_t section)
    -> sdbusplus::async::task<SectionDump>
{
    SectionDump dump{.section = section};

    if (!co_await session.bus().writeSingleRegister(
            config.address, blackbox.selectRegister, section))
    {
        co_return dump;
    }

    if (!co_await waitForSection(config, blackbox))
    {
        co_return dump;
    }

    // Which section is loaded is device state, so confirm it is still the one
    // asked for before reading the window.
    std::array<uint16_t, 1> selected{};
    if (!co_await session.bus().readHoldingRegisters(
            config.address, blackbox.selectRegister, selected) ||
        selected[0] != section)
    {
        co_return dump;
    }

    dump.raw =
        co_await readBlock(config, blackbox.dataRegister, blackbox.length);
    dump.read = !dump.raw.empty();
    co_return dump;
}

auto RegisterReader::readBlackbox(const ConfigIntf::Config& config,
                                  const ProfileIntf::Blackbox& blackbox)
    -> sdbusplus::async::task<std::vector<SectionDump>>
{
    std::vector<SectionDump> sections;

    if (blackbox.type != ProfileIntf::BlackboxType::fileRecord &&
        blackbox.type != ProfileIntf::BlackboxType::mailbox)
    {
        error("Unsupported blackbox type for {NAME}", "NAME", config.name);
        co_return sections;
    }

    for (auto section : blackbox.sections)
    {
        if (blackbox.type == ProfileIntf::BlackboxType::fileRecord)
        {
            sections.emplace_back(
                co_await readFileSection(config, section, blackbox.length));
        }
        else
        {
            sections.emplace_back(
                co_await readMailboxSection(config, blackbox, section));
        }
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
    auto probe = co_await session.probe(config, matched);
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
        decodeRegisters(config.profile, dump.registers);
        dump.result = Result::failure;
        dump.reason = "Probe value mismatch";
        co_return dump;
    }

    co_await readGroups(config, dump.registers);
    decodeRegisters(config.profile, dump.registers);

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
