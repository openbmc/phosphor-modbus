#pragma once

#include "base_config.hpp"
#include "modbus/modbus.hpp"
#include "port/base_port.hpp"

#include <sdbusplus/async.hpp>
#include <xyz/openbmc_project/Sensor/Value/aserver.hpp>

namespace phosphor::modbus::rtu::device
{

class Device;

using SensorValueIntf =
    sdbusplus::aserver::xyz::openbmc_project::sensor::Value<Device>;
using PortIntf = phosphor::modbus::rtu::port::BasePort;

class BaseDevice
{
  public:
    BaseDevice() = delete;

    explicit BaseDevice(sdbusplus::async::context& ctx,
                        const config::Config& config, PortIntf& serialPort);

    auto readSensorRegisters() -> sdbusplus::async::task<void>;

  private:
    auto createSensors() -> void;

    using sensors_map_t =
        std::unordered_map<std::string, std::unique_ptr<SensorValueIntf>>;
    sdbusplus::async::context& ctx;
    const config::Config config;
    PortIntf& serialPort;
    sensors_map_t sensors;
};

} // namespace phosphor::modbus::rtu::device
