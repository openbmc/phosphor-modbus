#include "cmd/dump.hpp"
#include "cmd/write.hpp"
#include "utils/common.hpp"
#include "utils/json_writer.hpp"
#include "utils/port_reservation.hpp"

#include <unistd.h>

#include <CLI/CLI.hpp>
#include <phosphor-logging/lg2/level.hpp>
#include <sdbusplus/async.hpp>

#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <expected>
#include <format>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{

using modbus_tool::Dump;
using modbus_tool::Result;

/** @brief Report on stderr what went wrong, so a failure is visible without
 *  reading the JSON.
 *  @return How many devices could not be read. */
auto reportFailures(const Dump& dump) -> size_t
{
    size_t unread = 0;
    for (const auto& device : dump.devices)
    {
        if (device.result == Result::failure)
        {
            unread++;
            std::cerr << device.name << ": " << device.reason << "\n";
        }
    }

    if (unread != 0)
    {
        std::cerr << unread << " of " << dump.devices.size()
                  << " devices could not be read\n";
    }

    return unread;
}

/** @brief Warn that the dump stops the daemon polling, and ask to go ahead.
 *  @return Whether to carry on. */
auto confirm() -> bool
{
    if (isatty(STDIN_FILENO) == 0)
    {
        std::cerr << "Refusing to run without confirmation; pass --yes\n";
        return false;
    }

    std::cerr << "This pauses " << modbus_tool::daemonService
              << " on the ports involved, so their\nsensors read as "
                 "unavailable until it finishes.\nContinue? [y/N] ";

    std::string answer;
    std::getline(std::cin, answer);
    return answer == "y" || answer == "Y";
}

auto write(const nlohmann::ordered_json& json, const std::string& path) -> bool
{
    if (path.empty())
    {
        std::cout << json.dump(2) << "\n";
        return true;
    }

    std::ofstream file(path);
    if (!file)
    {
        std::cerr << path << ": Cannot be written\n";
        return false;
    }
    file << json.dump(2) << "\n";
    return true;
}

struct Options
{
    std::vector<std::string> devices{};
    std::string output{};
    bool all = false;
    bool blackbox = false;
    bool assumeYes = false;
    bool verbose = false;

    std::string device{};
    std::string offset{};
    std::vector<std::string> values{};
};

/** @brief Parse a 16-bit register word, written in hex with or without 0x.
 *
 *  Offsets and values are hex throughout the tool, as a dump reports them,
 *  so there is no radix to guess at. */
auto parseHexWord(std::string_view text) -> std::optional<uint16_t>
{
    if (text.starts_with("0x") || text.starts_with("0X"))
    {
        text.remove_prefix(2);
    }

    uint16_t value{};
    const auto* end = text.data() + text.size();
    auto [stopped, ec] = std::from_chars(text.data(), end, value, 16);
    if (ec != std::errc{} || stopped != end)
    {
        return std::nullopt;
    }
    return value;
}

/** @brief Define the dump subcommand and what it takes. */
auto addDumpCommand(CLI::App& app, Options& options) -> void
{
    auto* dump = app.add_subcommand("dump", "Dump device registers");
    auto* named = dump->add_option("-d,--devices", options.devices,
                                   "Comma separated list of devices to dump");
    named->delimiter(',');
    dump->add_flag("-a,--all", options.all,
                   "Dump every device the platform allows")
        ->excludes(named);
    dump->add_flag("-b,--blackbox", options.blackbox, "Also read the blackbox");
    dump->add_option("-o,--output", options.output,
                     "Write the JSON here instead of stdout");
    dump->add_flag("-y,--yes", options.assumeYes,
                   "Do not ask before pausing monitoring");
    dump->add_flag("-v,--verbose", options.verbose,
                   "Log everything the read does");
}

/** @brief Define the write subcommand and what it takes. */
auto addWriteCommand(CLI::App& app, Options& options) -> void
{
    auto* write = app.add_subcommand("write", "Write a device's registers");
    write->add_option("-d,--device", options.device, "Device to write to")
        ->required();
    write
        ->add_option("--offset", options.offset,
                     "Register offset in hex, e.g. 5A or 0x5A")
        ->required();
    auto* values = write->add_option(
        "--value", options.values,
        "Register value in hex, comma separated for multiple registers");
    values->delimiter(',');
    values->required();
    write->add_flag("-y,--yes", options.assumeYes,
                    "Do not ask before pausing monitoring");
    write->add_flag("-v,--verbose", options.verbose,
                    "Log everything the write does");
}

/** @brief Perform the write and report how it ended. */
auto takeWrite(const std::string& name, uint16_t offset,
               const std::vector<uint16_t>& values) -> modbus_tool::WriteStatus
{
    sdbusplus::async::context ctx;

    auto status = modbus_tool::WriteStatus::portUnavailable;
    ctx.spawn(modbus_tool::runWrite(ctx, name, offset, values) |
              sdbusplus::async::execution::then(
                  [&](modbus_tool::WriteStatus written) {
                      status = written;
                      ctx.request_stop();
                  }));
    ctx.run();

    return status;
}

/** @brief Produce the dump, resolving --all to a device list first.
 *  @return The dump, or why there was nothing to attempt. */
auto takeDump(std::vector<std::string> devices, bool all, bool withBlackbox)
    -> std::expected<Dump, std::string>
{
    sdbusplus::async::context ctx;

    if (all)
    {
        auto allowed = modbus_tool::allowedDeviceNames(ctx, CONFIG_DIR);
        if (!allowed)
        {
            return std::unexpected(allowed.error());
        }
        devices = std::move(*allowed);
    }

    Dump result;
    ctx.spawn(modbus_tool::runDump(ctx, devices, withBlackbox) |
              sdbusplus::async::execution::then([&](Dump dumped) {
                  result = std::move(dumped);
                  ctx.request_stop();
              }));
    ctx.run();

    return result;
}

/** @brief Run the dump subcommand. */
auto runDumpCommand(Options& options) -> int
{
    // --all excludes --devices, so only neither being given is left to catch.
    if (options.devices.empty() && !options.all)
    {
        std::cerr << "Name the devices with --devices, or pass --all\n";
        return 1;
    }

    if (!options.assumeYes && !confirm())
    {
        return 1;
    }

    // Only one instance may run at a time.
    modbus_tool::InstanceLock lock;
    if (!lock.acquire())
    {
        std::cerr << "Another modbus-tool is already running\n";
        return 1;
    }

    auto result =
        takeDump(std::move(options.devices), options.all, options.blackbox);
    if (!result)
    {
        std::cerr << result.error() << "\n";
        return 1;
    }

    auto failures = reportFailures(*result);
    if (!write(modbus_tool::toJson(*result), options.output))
    {
        return 1;
    }

    // A dump where nothing could be read is of no use.
    return failures == result->devices.size() ? 1 : 0;
}

/** @brief Run the write subcommand. */
auto runWriteCommand(const Options& options) -> int
{
    auto offset = parseHexWord(options.offset);
    if (!offset)
    {
        std::cerr << "Not a register offset: " << options.offset << "\n";
        return 1;
    }

    std::vector<uint16_t> values;
    for (const auto& text : options.values)
    {
        auto value = parseHexWord(text);
        if (!value)
        {
            std::cerr << "Not a register value: " << text << "\n";
            return 1;
        }
        values.emplace_back(*value);
    }

    // Say what is about to be written before asking, so a wrong device,
    // offset or value is visible while it can still be stopped.
    std::cerr << options.device << ": write";
    for (auto value : values)
    {
        std::cerr << std::format(" {:#06x}", value);
    }
    std::cerr << std::format(" to offset {:#06x}\n", *offset);

    if (!options.assumeYes && !confirm())
    {
        return 1;
    }

    // Only one instance may run at a time.
    modbus_tool::InstanceLock lock;
    if (!lock.acquire())
    {
        std::cerr << "Another modbus-tool is already running\n";
        return 1;
    }

    auto status = takeWrite(options.device, *offset, values);
    auto ok = status == modbus_tool::WriteStatus::success;

    std::cerr << options.device << ": "
              << modbus_tool::writeStatusMessage(status) << "\n";
    std::cout << nlohmann::ordered_json{{"Result", ok ? "Success" : "Failure"}}
                     .dump(2)
              << "\n";

    return ok ? 0 : 1;
}

} // namespace

int main(int argc, char** argv)
{
    CLI::App app{"Read and write a device's registers."};
    app.require_subcommand(1);

    Options options;
    addDumpCommand(app, options);
    addWriteCommand(app, options);

    CLI11_PARSE(app, argc, argv);

    // lg2 mirrors every level to stderr on a terminal, which buries the
    // output. Leave a level the caller set alone.
    if (!options.verbose)
    {
        auto quiet = std::to_string(std::to_underlying(lg2::level::warning));
        setenv("LG2_LOG_LEVEL", quiet.c_str(), 0);
    }

    if (app.got_subcommand("write"))
    {
        return runWriteCommand(options);
    }

    return runDumpCommand(options);
}
