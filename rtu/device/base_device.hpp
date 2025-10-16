#pragma once

#include "base_config.hpp"
#include "common/events.hpp"
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
namespace EventIntf = phosphor::modbus::events;

class BaseDevice
{
  public:
    BaseDevice() = delete;

    explicit BaseDevice(sdbusplus::async::context& ctx,
                        const config::Config& config, PortIntf& serialPort,
                        EventIntf::Events& events);

    auto readSensorRegisters() -> sdbusplus::async::task<void>;

  private:
    auto createSensors() -> void;

    auto readStatusRegisters() -> sdbusplus::async::task<void>;

    auto generateEvent(const config::StatusBit& statusBit,
                       const sdbusplus::message::object_path& objectPath,
                       double sensorValue, SensorValueIntf::Unit sensorUnit,
                       bool statusAsserted) -> sdbusplus::async::task<void>;

    using sensors_map_t =
        std::unordered_map<std::string, std::unique_ptr<SensorValueIntf>>;
    sdbusplus::async::context& ctx;
    const config::Config config;
    PortIntf& serialPort;
    EventIntf::Events& events;
    sensors_map_t sensors;
};

} // namespace phosphor::modbus::rtu::device
