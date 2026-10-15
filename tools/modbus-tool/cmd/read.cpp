#include "cmd/read.hpp"

#include "utils/entity_manager_lookup.hpp"
#include "utils/port_reservation.hpp"
#include "utils/serial_session.hpp"

#include <phosphor-logging/lg2.hpp>

#include <format>
#include <utility>
#include <vector>

namespace modbus_tool
{

PHOSPHOR_LOG2_USING;

namespace
{

/** @brief Read from the variant of a device that answered its probe.
 *  @param[out] registers What the device returned, left empty on failure. */
auto readProbed(SerialSession& session, const DeviceVariants& device,
                uint16_t offset, uint16_t size, const std::string& portName,
                std::vector<uint16_t>& registers)
    -> sdbusplus::async::task<ReadStatus>
{
    auto probed = co_await session.probeVariants(device);
    if (probed.config == nullptr)
    {
        co_return probed.answered ? ReadStatus::probeMismatch
                                  : ReadStatus::noResponse;
    }

    // The probe loop may have left another variant's settings on the port,
    // so put the ones this device wants back.
    const auto& config = *probed.config;
    if (!session.applyProfile(config))
    {
        co_return ReadStatus::failed;
    }

    std::vector<uint16_t> read(size);
    if (!co_await session.readHoldingRegisters(config.address, offset, read))
    {
        co_return ReadStatus::failed;
    }
    registers = std::move(read);

    info("Read {COUNT} register(s) at {OFFSET} on {NAME} {ADDRESS} {PORT}",
         "COUNT", size, "OFFSET", lg2::hex, offset, "NAME", config.name,
         "ADDRESS", lg2::hex, config.address, "PORT", portName);

    co_return ReadStatus::success;
}

} // namespace

auto readStatusMessage(ReadStatus status) -> std::string_view
{
    switch (status)
    {
        case ReadStatus::success:
            return "Read";
        case ReadStatus::notConfigured:
            return "Not configured";
        case ReadStatus::portUnavailable:
            return "Port unavailable";
        case ReadStatus::noResponse:
            return "No response";
        case ReadStatus::probeMismatch:
            return "Probe value mismatch";
        case ReadStatus::failed:
            return "Read failed";
    }
    return "Read failed";
}

auto toJson(const ReadResult& result) -> nlohmann::ordered_json
{
    nlohmann::ordered_json out;
    auto success = result.status == ReadStatus::success;
    out["Result"] = success ? "Success" : "Failure";
    if (!success)
    {
        out["Reason"] = readStatusMessage(result.status);
    }
    out["Offset"] = std::format("0x{:X}", result.offset);
    out["Size"] = result.size;

    auto raw = nlohmann::ordered_json::array();
    for (auto word : result.registers)
    {
        raw.emplace_back(std::format("0x{:04X}", word));
    }
    out["Raw"] = std::move(raw);

    return out;
}

auto runRead(sdbusplus::async::context& ctx, const std::string& name,
             uint16_t offset, uint16_t size)
    -> sdbusplus::async::task<ReadResult>
{
    auto devices = co_await lookupDevices(ctx, {name});
    if (devices.empty() || devices.front().configs.empty())
    {
        co_return ReadResult{.status = ReadStatus::notConfigured,
                             .offset = offset,
                             .size = size};
    }

    co_return co_await readDevice(ctx, devices.front(), offset, size,
                                  lookupPort);
}

auto readDevice(sdbusplus::async::context& ctx, const DeviceVariants& device,
                uint16_t offset, uint16_t size, const PortLookup& lookupPortFn)
    -> sdbusplus::async::task<ReadResult>
{
    ReadResult result{.offset = offset, .size = size};

    if (device.configs.empty())
    {
        result.status = ReadStatus::notConfigured;
        co_return result;
    }

    // Every variant of a device sits on the same port.
    const auto& portName = device.configs.front().serialPort;

    auto port = co_await lookupPortFn(ctx, portName);
    if (!port.config)
    {
        co_return result;
    }

    PortReservation reservation(portName);
    if (!co_await reservation.reserve(ctx))
    {
        co_return result;
    }

    try
    {
        SerialSession session(ctx, *port.config, port.devicePath);
        result.status = co_await readProbed(session, device, offset, size,
                                            portName, result.registers);
    }
    catch (const std::exception& e)
    {
        error("Cannot read {PORT}: {ERROR}", "PORT", portName, "ERROR", e);
    }

    co_await reservation.release(ctx);
    co_return result;
}

} // namespace modbus_tool
