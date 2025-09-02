#include "device_firmware.hpp"

#include "device.hpp"

#include <phosphor-logging/lg2.hpp>

namespace phosphor::modbus::rtu::device
{

PHOSPHOR_LOG2_USING;

static auto getRandomId() -> long int
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    unsigned int seed = ts.tv_nsec ^ getpid();
    srandom(seed);
    return random() % 10000;
}

static auto getObjectPath(const config_intf::Config& config)
    -> sdbusplus::message::object_path
{
    for (const auto& firmwareRegister : config.firmwareRegisters)
    {
        if (firmwareRegister.type == config_intf::FirmwareRegisterType::version)
        {
            return sdbusplus::message::object_path(
                       FirmwareIntf::namespace_path) /
                   std::format("{}_{}", firmwareRegister.name, getRandomId());
        }
    }

    return sdbusplus::message::object_path();
}

constexpr FirmwareIntf::Version::properties_t initVersion{
    "Unknown", FirmwareIntf::VersionPurpose::Other};
constexpr FirmwareIntf::Activation::properties_t initActivation{
    FirmwareIntf::Activations::NotReady,
    FirmwareIntf::RequestedActivations::None};
constexpr FirmwareIntf::Definitions::properties_t initAssociations{};

DeviceFirmware::DeviceFirmware(sdbusplus::async::context& ctx,
                               const config_intf::Config& config,
                               const std::unique_ptr<PortIntf>& serialPort) :
    FirmwareIntf(ctx, getObjectPath(config).str.c_str(), initVersion,
                 initActivation, initAssociations),
    ctx(ctx), config(config), serialPort(serialPort)
{
    ctx.spawn(readVersionRegister());
}

auto DeviceFirmware::readVersionRegister() -> sdbusplus::async::task<void>
{
    const auto it = std::find_if(
        config.firmwareRegisters.begin(), config.firmwareRegisters.end(),
        [](const config_intf::FirmwareRegister& firmwareRegister) {
            return firmwareRegister.type ==
                   config_intf::FirmwareRegisterType::version;
        });

    if (it == config.firmwareRegisters.end())
    {
        error("No firmware version register found for {NAME}", "NAME",
              config.name);
        co_return;
    }

    const config_intf::FirmwareRegister& versionRegister = *it;

    auto registers = std::vector<uint16_t>(versionRegister.size);
    auto ret = co_await serialPort->readHoldingRegisters(
        config.address, versionRegister.offset, config.baudRate, config.parity,
        registers);
    if (!ret)
    {
        error("Failed to read holding registers {NAME} for {DEVICE_ADDRESS}",
              "NAME", versionRegister.name, "DEVICE_ADDRESS", config.address);
        co_return;
    }

    std::string strValue = "";

    for (const auto& value : registers)
    {
        strValue += static_cast<char>((value >> 8) & 0xFF);
        strValue += static_cast<char>(value & 0xFF);
    }

    version(strValue);
    activation(Activation::Activations::Active);
    auto associationList =
        std::vector<std::tuple<std::string, std::string, std::string>>{
            {"running", "ran_on", config.inventoryPath}};
    associations(associationList);

    Version::emit_added();
    Activation::emit_added();
    Definitions::emit_added();
}

} // namespace phosphor::modbus::rtu::device
