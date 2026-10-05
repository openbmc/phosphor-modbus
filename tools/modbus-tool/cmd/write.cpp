#include "cmd/write.hpp"

#include "utils/entity_manager_lookup.hpp"
#include "utils/port_reservation.hpp"
#include "utils/serial_session.hpp"

#include <phosphor-logging/lg2.hpp>

#include <vector>

namespace modbus_tool
{

PHOSPHOR_LOG2_USING;

namespace
{

/** @brief The variant of a device that answered its probe.
 *
 *  A second sourced device carries a configuration per variant and only one
 *  is really present, so the write needs to know which answered before it
 *  can address one. */
struct ProbeOutcome
{
    const ConfigIntf::Config* config = nullptr;
    bool answered = false;
};

/** @brief Probe each variant and keep the one that is present. */
auto probeVariants(SerialSession& session, const DeviceVariants& device)
    -> sdbusplus::async::task<ProbeOutcome>
{
    ProbeOutcome outcome;

    for (const auto& config : device.configs)
    {
        bool matched = false;
        auto registers = co_await session.probe(config, matched);
        if (registers.empty())
        {
            continue;
        }

        outcome.answered = true;
        if (matched)
        {
            outcome.config = &config;
            break;
        }
    }

    co_return outcome;
}

/** @brief Write to the variant of a device that answered its probe. */
auto writeProbed(SerialSession& session, const DeviceVariants& device,
                 uint16_t offset, std::span<const uint16_t> values,
                 const std::string& portName)
    -> sdbusplus::async::task<WriteStatus>
{
    auto probed = co_await probeVariants(session, device);
    if (probed.config == nullptr)
    {
        co_return probed.answered ? WriteStatus::probeMismatch
                                  : WriteStatus::noResponse;
    }

    // The probe loop may have left another variant's settings on the port,
    // so put the ones this device wants back.
    const auto& config = *probed.config;
    if (!session.applyProfile(config) ||
        !co_await session.writeMultipleRegisters(config.address, offset,
                                                 values))
    {
        co_return WriteStatus::rejected;
    }

    // A write changes the device, so leave a record of it behind. Logged
    // above the level the tool quiets itself to, so it is in the journal
    // without asking for it.
    warning("Wrote {COUNT} register(s) at {OFFSET} on {NAME} {ADDRESS} {PORT}",
            "COUNT", values.size(), "OFFSET", lg2::hex, offset, "NAME",
            config.name, "ADDRESS", lg2::hex, config.address, "PORT", portName);

    co_return WriteStatus::success;
}

} // namespace

auto writeStatusMessage(WriteStatus status) -> std::string_view
{
    switch (status)
    {
        case WriteStatus::success:
            return "Written";
        case WriteStatus::notConfigured:
            return "Not configured";
        case WriteStatus::portUnavailable:
            return "Port unavailable";
        case WriteStatus::noResponse:
            return "No response";
        case WriteStatus::probeMismatch:
            return "Probe value mismatch";
        case WriteStatus::rejected:
            return "Write rejected";
    }
    return "Write rejected";
}

auto runWrite(sdbusplus::async::context& ctx, const std::string& name,
              uint16_t offset, std::span<const uint16_t> values)
    -> sdbusplus::async::task<WriteStatus>
{
    auto devices = co_await lookupDevices(ctx, {name});
    if (devices.empty() || devices.front().configs.empty())
    {
        co_return WriteStatus::notConfigured;
    }

    co_return co_await writeDevice(ctx, devices.front(), offset, values,
                                   lookupPort);
}

auto writeDevice(sdbusplus::async::context& ctx, const DeviceVariants& device,
                 uint16_t offset, std::span<const uint16_t> values,
                 const PortLookup& lookupPortFn)
    -> sdbusplus::async::task<WriteStatus>
{
    if (device.configs.empty())
    {
        co_return WriteStatus::notConfigured;
    }

    // Every variant of a device sits on the same port.
    const auto& portName = device.configs.front().serialPort;

    auto port = co_await lookupPortFn(ctx, portName);
    if (!port.config)
    {
        co_return WriteStatus::portUnavailable;
    }

    PortReservation reservation(portName);
    if (!co_await reservation.reserve(ctx))
    {
        co_return WriteStatus::portUnavailable;
    }

    auto status = WriteStatus::portUnavailable;
    try
    {
        SerialSession session(ctx, *port.config, port.devicePath);
        status =
            co_await writeProbed(session, device, offset, values, portName);
    }
    catch (const std::exception& e)
    {
        error("Cannot write {PORT}: {ERROR}", "PORT", portName, "ERROR", e);
    }

    co_await reservation.release(ctx);
    co_return status;
}

} // namespace modbus_tool
