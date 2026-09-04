#include "cmd/dump.hpp"
#include "utils/common.hpp"
#include "utils/json_writer.hpp"
#include "utils/port_reservation.hpp"

#include <unistd.h>

#include <CLI/CLI.hpp>
#include <sdbusplus/async.hpp>

#include <cstddef>
#include <fstream>
#include <iostream>
#include <string>
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

auto write(const nlohmann::json& json, const std::string& path) -> bool
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

} // namespace

int main(int argc, char** argv)
{
    CLI::App app{"Read a device's registers and write them to JSON."};
    app.require_subcommand(1);

    std::vector<std::string> devices;
    std::string output;
    bool assumeYes = false;

    auto* dump = app.add_subcommand("dump", "Dump device registers");
    dump->add_option("--devices", devices,
                     "Comma separated list of devices to dump")
        ->required()
        ->delimiter(',');
    dump->add_option("--output", output,
                     "Write the JSON here instead of stdout");
    dump->add_flag("-y,--yes", assumeYes,
                   "Do not ask before pausing monitoring");

    CLI11_PARSE(app, argc, argv);

    if (!assumeYes && !confirm())
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

    Dump result;
    sdbusplus::async::context ctx;
    ctx.spawn(modbus_tool::runDump(ctx, devices) |
              sdbusplus::async::execution::then([&](Dump dumped) {
                  result = std::move(dumped);
                  ctx.request_stop();
              }));
    ctx.run();

    auto failures = reportFailures(result);
    if (!write(modbus_tool::toJson(result), output))
    {
        return 1;
    }

    // A dump where nothing could be read is of no use.
    return failures == result.devices.size() ? 1 : 0;
}
