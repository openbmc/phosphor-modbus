#pragma once

#include "device/base_config.hpp"
#include "port/base_port.hpp"

#include <sdbusplus/async.hpp>
#include <sdbusplus/async/server.hpp>
#include <xyz/openbmc_project/Association/Definitions/aserver.hpp>
#include <xyz/openbmc_project/Software/Activation/aserver.hpp>
#include <xyz/openbmc_project/Software/Version/aserver.hpp>

namespace phosphor::modbus::rtu::device
{

namespace config
{

struct Config;

} // namespace config

namespace config_intf = phosphor::modbus::rtu::device::config;
using PortIntf = phosphor::modbus::rtu::port::BasePort;

class DeviceFirmware;

using FirmwareIntf = sdbusplus::async::server_t<
    DeviceFirmware, sdbusplus::aserver::xyz::openbmc_project::software::Version,
    sdbusplus::aserver::xyz::openbmc_project::software::Activation,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

class DeviceFirmware
{
  public:
    DeviceFirmware() = delete;

    explicit DeviceFirmware(sdbusplus::async::context& ctx,
                            const config_intf::Config& config,
                            PortIntf& serialPort);

    auto readVersionRegister() -> sdbusplus::async::task<void>;

  protected:
    // Object path of current firmware object
    // TODO: check if its possible to get rid off this via mocking since its
    // only used in tests
    const sdbusplus::message::object_path objectPath;

  private:
    std::unique_ptr<FirmwareIntf> currentFirmware;
    const config_intf::Config config;
    PortIntf& serialPort;
};

} // namespace phosphor::modbus::rtu::device
