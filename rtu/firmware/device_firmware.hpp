#pragma once

#include "serial_port.hpp"

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
using SerialPortIntf = phosphor::modbus::rtu::serial_port::SerialPort;

class DeviceFirmware;

using FirmwareIntf = sdbusplus::async::server_t<
    DeviceFirmware, sdbusplus::aserver::xyz::openbmc_project::software::Version,
    sdbusplus::aserver::xyz::openbmc_project::software::Activation,
    sdbusplus::aserver::xyz::openbmc_project::association::Definitions>;

class DeviceFirmware : public FirmwareIntf
{
  public:
    DeviceFirmware() = delete;

    explicit DeviceFirmware(sdbusplus::async::context& ctx,
                            const config_intf::Config& config,
                            const std::unique_ptr<SerialPortIntf>& serialPort);

  private:
    auto readVersionRegister() -> sdbusplus::async::task<void>;

    sdbusplus::async::context& ctx;
    const config_intf::Config& config;
    const std::unique_ptr<SerialPortIntf>& serialPort;
};

} // namespace phosphor::modbus::rtu::device
