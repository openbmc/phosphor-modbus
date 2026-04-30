#include "modbus_inventory.hpp"

#include "modbus_rtu_config.hpp"

#include <phosphor-logging/lg2.hpp>

#include <type_traits>
#include <variant>

namespace phosphor::modbus::rtu::inventory
{
PHOSPHOR_LOG2_USING;
namespace ProfileIntf = phosphor::modbus::rtu::profile;

static auto fillAssetProperties(
    InventoryServer::AssetIntf::properties_t& properties,
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

static auto matchesProbeValue(const std::vector<uint16_t>& readBuffer,
                              const ProfileIntf::ProbeRegister& probe) -> bool
{
    return std::visit(
        [&readBuffer](const auto& expected) -> bool {
            using T = std::decay_t<decltype(expected)>;
            if constexpr (std::is_same_v<T, uint64_t>)
            {
                uint64_t value = 0;
                for (const auto& reg : readBuffer)
                {
                    value = (value << 16) | reg;
                }
                return value == expected;
            }
            else // std::string
            {
                std::string value;
                for (const auto& reg : readBuffer)
                {
                    value += static_cast<char>((reg >> 8) & 0xFF);
                    value += static_cast<char>(reg & 0xFF);
                }
                // Remove null characters
                std::erase(value, '\0');
                return value == expected;
            }
        },
        probe.expectedValue);
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
    while (isRunning())
    {
        co_await probeDevice();

        // Sleep in short intervals so we can respond to stop requests
        // promptly instead of blocking for the full probe interval.
        constexpr auto stopCheckInterval = std::chrono::seconds(3);
        for (auto elapsed = std::chrono::seconds(0);
             elapsed < inventoryProbeInterval && isRunning();
             elapsed += stopCheckInterval)
        {
            co_await sdbusplus::async::sleep_for(ctx, stopCheckInterval);
        }
    }

    // Clean up inventory D-Bus object if it exists
    if (inventoryServer)
    {
        inventoryServer->emit_removed();
        inventoryServer.reset();
        if (probeCallback)
        {
            co_await probeCallback(false);
        }
    }
    stopped = true;
}

auto Device::isRunning() const -> bool
{
    return !ctx.stop_requested() && !stopRequested;
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

auto Device::handleProbeFailed() -> sdbusplus::async::task<void>
{
    if (inventoryServer)
    {
        warning("Device {NAME} removed at {ADDRESS} due to probe failure",
                "NAME", config.name, "ADDRESS", config.address);
        inventoryServer->emit_removed();
        inventoryServer.reset();
        if (probeCallback)
        {
            co_await probeCallback(false);
        }
    }
    else
    {
        dormant = true;
        dormantSince = std::chrono::steady_clock::now();
        debug("Device {NAME} at {ADDRESS} marked dormant", "NAME", config.name,
              "ADDRESS", config.address);
    }
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

    const auto& probe = config.profile.probeRegister;
    auto registers = std::vector<uint16_t>(probe.size);

    auto ret = co_await port.readHoldingRegisters(
        config.address, probe.offset, config.profile.baudRate,
        config.profile.parity, registers);
    if (!ret)
    {
        co_await handleProbeFailed();
        co_return;
    }

    if (!matchesProbeValue(registers, probe))
    {
        if (!mismatchLogged)
        {
            warning("Device {NAME} at {ADDRESS} probe value mismatch, "
                    "expected device not present",
                    "NAME", config.name, "ADDRESS", config.address);
            mismatchLogged = true;
        }
        dormant = true;
        dormantSince = std::chrono::steady_clock::now();
        co_return;
    }

    if (!inventoryServer)
    {
        debug("Device {NAME} found at {ADDRESS}", "NAME", config.name,
              "ADDRESS", config.address);
        mismatchLogged = false;
        co_await addInventoryServer();
        if (probeCallback)
        {
            co_await probeCallback(true);
        }
    }
}

auto Device::addInventoryServer() -> sdbusplus::async::task<void>
{
    InventoryServer::AssetIntf::properties_t assetProps;

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

            fillAssetProperties(assetProps, reg.type, strValue);
        }
    }

    InventoryServer::ChassisIntf::properties_t chassisProps{
        .type = InventoryServer::ChassisIntf::ChassisType::Module};

    InventoryServer::AssocIntf::properties_t assocProps{};

    auto pathSuffix = config.name + "_" + std::to_string(config.address) + "_" +
                      config.serialPort;

    auto objectPath = std::string(inventoryServerPath) + "/" + pathSuffix;

    inventoryServer = std::make_unique<InventoryServer>(
        ctx, objectPath.c_str(), chassisProps, assetProps, assocProps);
    inventoryServer->emit_added();

    info("Added inventory object at {PATH}", "PATH", objectPath);
}

} // namespace phosphor::modbus::rtu::inventory
