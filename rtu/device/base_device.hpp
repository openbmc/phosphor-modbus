#pragma once

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
namespace ModbusIntf = phosphor::modbus::rtu;

namespace config
{

enum class SensorFormat
{
    floatingPoint,
    integer,
    unknown
};

struct SensorRegister
{
    std::string name = "unknown";
    std::string pathSuffix = "unknown";
    SensorValueIntf::Unit unit;
    uint16_t offset = 0;
    uint8_t size = 0;
    uint8_t precision = 0;
    double scale = 1.0;
    double shift = 0.0;
    bool isSigned = false;
    SensorFormat format = SensorFormat::unknown;
};

enum class StatusType
{
    controllerFailure,
    fanFailure,
    filterFailure,
    powerFault,
    pumpFailure,
    leakDetectedCritical,
    leakDetectedWarning,
    sensorFailure,
    sensorReadingCritical,
    sensorReadingWarning,
    unknown
};

struct StatusBit
{
    std::string name = "unknown";
    StatusType type = StatusType::unknown;
    uint8_t bitPostion = 0;
    bool value = false;
};

enum class FirmwareRegisterType
{
    version,
    update,
    unknown
};

struct FirmwareRegister
{
    std::string name = "unknown";
    FirmwareRegisterType type = FirmwareRegisterType::unknown;
    uint16_t offset = 0;
    uint8_t size = 0;
};

struct Config
{
    using sensor_registers_t = std::vector<SensorRegister>;
    using status_registers_t =
        std::unordered_map<uint16_t, std::vector<StatusBit>>;
    using firmware_registers_t = std::vector<FirmwareRegister>;

    uint8_t address = 0;
    ModbusIntf::Parity parity = ModbusIntf::Parity::unknown;
    uint32_t baudRate = 0;
    std::string name = "unknown";
    std::string portName = "unknown";
    sdbusplus::message::object_path inventoryPath;
    sensor_registers_t sensorRegisters;
    status_registers_t statusRegisters;
    firmware_registers_t firmwareRegisters;
};

auto updateBaseConfig(sdbusplus::async::context& ctx,
                      const sdbusplus::message::object_path& objectPath,
                      const std::string& interfaceName, Config& config)
    -> sdbusplus::async::task<bool>;

} // namespace config

class BaseDevice
{
  public:
    BaseDevice() = delete;

    explicit BaseDevice(sdbusplus::async::context& ctx,
                        const config::Config& config, PortIntf& serialPort);

    auto createSensors() -> sdbusplus::async::task<void>;

  private:
    auto readSensorRegisters() -> sdbusplus::async::task<void>;

    using sensors_map_t =
        std::unordered_map<std::string, std::unique_ptr<SensorValueIntf>>;
    sdbusplus::async::context& ctx;
    const config::Config config;
    PortIntf& serialPort;
    sensors_map_t sensors;
};

} // namespace phosphor::modbus::rtu::device
