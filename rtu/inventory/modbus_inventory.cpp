#include "modbus_inventory.hpp"

#include "modbus_rtu_config.hpp"

#include <phosphor-logging/lg2.hpp>

namespace phosphor::modbus::rtu::inventory
{
PHOSPHOR_LOG2_USING;
namespace ProfileIntf = phosphor::modbus::rtu::profile;

namespace ProfileIntf = phosphor::modbus::rtu::profile;

static auto fillInventorySourceProperties(
    InventorySourceIntf::properties_t& properties,
    ProfileIntf::InventoryDataType type, std::string& value) -> void
{
    switch (type)
    {
        case ProfileIntf::InventoryDataType::partNumber:
            properties.part_number = value;
            break;
        case ProfileIntf::InventoryDataType::sparePartNumber:
            properties.spare_part_number = value;
            break;
        case ProfileIntf::InventoryDataType::serialNumber:
            properties.serial_number = value;
            break;
        case ProfileIntf::InventoryDataType::buildDate:
            properties.build_date = value;
            break;
        case ProfileIntf::InventoryDataType::model:
            properties.model = value;
            break;
        case ProfileIntf::InventoryDataType::manufacturer:
            properties.manufacturer = value;
            break;
        case ProfileIntf::InventoryDataType::unknown:
            error("Unknown inventory data type");
            break;
    }
}

Device::Device(sdbusplus::async::context& ctx, const config::Config& config,
               SerialPortIntf& port, ProbeCallback probeCallback,
               std::chrono::seconds dormantPeriod) :
    config(config), ctx(ctx), port(port),
    probeCallback(std::move(probeCallback)), dormantPeriod([&]() {
        if (dormantPeriod > std::chrono::seconds(0))
        {
            return dormantPeriod;
        }
        // Ceiling division to round up to the nearest multiple of the
        // probe interval. std::chrono::ceil only converts between duration
        // types and cannot compute multiples of an arbitrary interval.
        auto multiplier = (inventoryDormantProbeInterval +
                           inventoryProbeInterval - std::chrono::seconds(1)) /
                          inventoryProbeInterval;
        return multiplier * inventoryProbeInterval;
    }())
{
    std::vector<RegisterInfo> regInfos;
    regInfos.reserve(config.profile.inventoryRegisters.size());
    for (const auto& reg : config.profile.inventoryRegisters)
    {
        regInfos.push_back({.offset = reg.offset, .size = reg.size});
    }
    registerSpans = buildRegisterSpans(regInfos, maxRegisterSpanLength);
}

auto Device::startProbing() -> sdbusplus::async::task<void>
{
    debug("Probing device {NAME} at address {ADDRESS} on port {PORT}", "NAME",
          config.name, "ADDRESS", config.address, "PORT", config.serialPort);
    while (!ctx.stop_requested())
    {
        co_await probeDevice();

        // Sleep in short intervals so we can respond to stop requests
        // promptly instead of blocking for the full probe interval.
        constexpr auto stopCheckInterval = std::chrono::seconds(3);
        for (auto elapsed = std::chrono::seconds(0);
             elapsed < inventoryProbeInterval && !ctx.stop_requested();
             elapsed += stopCheckInterval)
        {
            co_await sdbusplus::async::sleep_for(ctx, stopCheckInterval);
        }
    }
}

auto Device::checkAndClearDormant() -> bool
{
    if (!dormant)
    {
        return false;
    }
    auto elapsed = std::chrono::steady_clock::now() - dormantSince;
    if (elapsed < dormantPeriod)
    {
        return true;
    }
    dormant = false;
    return false;
}

auto Device::probeDevice() -> sdbusplus::async::task<void>
{
    if (checkAndClearDormant())
    {
        debug("Device {NAME} at {ADDRESS} is dormant, skipping", "NAME",
              config.name, "ADDRESS", config.address);
        co_return;
    }

    debug("Probing device {NAME} at {ADDRESS}", "NAME", config.name, "ADDRESS",
          config.address);

    if (config.profile.inventoryRegisters.empty())
    {
        error("No inventory registers configured for {NAME}", "NAME",
              config.name);
        co_return;
    }
    auto probeRegister = config.profile.inventoryRegisters[0].offset;
    auto registers =
        std::vector<uint16_t>(config.profile.inventoryRegisters[0].size);

    auto ret = co_await port.readHoldingRegisters(
        config.address, probeRegister, config.profile.baudRate,
        config.profile.parity, registers);
    if (ret)
    {
        if (!inventorySource)
        {
            debug("Device {NAME} found at {ADDRESS}", "NAME", config.name,
                  "ADDRESS", config.address);
            co_await addInventorySource();
            if (probeCallback)
            {
                co_await probeCallback(true);
            }
        }
    }
    else
    {
        if (inventorySource)
        {
            warning("Device {NAME} removed at {ADDRESS} due to probe failure",
                    "NAME", config.name, "ADDRESS", config.address);
            inventorySource->emit_removed();
            inventorySource.reset();
            if (probeCallback)
            {
                co_await probeCallback(false);
            }
        }
        else
        {
            dormant = true;
            dormantSince = std::chrono::steady_clock::now();
            debug("Device {NAME} at {ADDRESS} marked dormant", "NAME",
                  config.name, "ADDRESS", config.address);
        }
    }
}

auto Device::addInventorySource() -> sdbusplus::async::task<void>
{
    InventorySourceIntf::properties_t properties;

    for (const auto& span : registerSpans)
    {
        auto readBuffer = std::vector<uint16_t>(span.totalSize);
        auto ret = co_await port.readHoldingRegisters(
            config.address, span.startOffset, config.profile.baudRate,
            config.profile.parity, readBuffer);
        if (!ret)
        {
            for (auto idx : span.registerIndices)
            {
                error(
                    "Failed to read holding registers {NAME} for {DEVICE_ADDRESS}",
                    "NAME", config.profile.inventoryRegisters[idx].name,
                    "DEVICE_ADDRESS", config.address);
            }
            continue;
        }

        for (auto idx : span.registerIndices)
        {
            const auto& reg = config.profile.inventoryRegisters[idx];
            auto regStart = reg.offset - span.startOffset;
            std::string strValue = "";

            // Reswap bytes in each register for string conversion
            for (uint8_t i = 0; i < reg.size; i++)
            {
                auto value = readBuffer[regStart + i];
                strValue += static_cast<char>((value >> 8) & 0xFF);
                strValue += static_cast<char>(value & 0xFF);
            }

            fillInventorySourceProperties(properties, reg.type, strValue);
        }
    }

    auto pathSuffix = config.name + " " + std::to_string(config.address) + " " +
                      config.serialPort;

    properties.name = pathSuffix;
    properties.address = config.address;
    properties.link_tty = config.serialPort;

    std::replace(pathSuffix.begin(), pathSuffix.end(), ' ', '_');

    auto objectPath =
        std::string(InventorySourceIntf::namespace_path) + "/" + pathSuffix;

    inventorySource = std::make_unique<InventorySourceIntf>(
        ctx, objectPath.c_str(), properties);
    inventorySource->emit_added();

    info("Added InventorySource at {PATH}", "PATH", objectPath);
}

} // namespace phosphor::modbus::rtu::inventory
