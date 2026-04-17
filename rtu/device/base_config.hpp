#pragma once

#include "device_profile.hpp"
#include "modbus/modbus.hpp"

#include <chrono>

namespace phosphor::modbus::rtu::device
{

namespace ModbusIntf = phosphor::modbus::rtu;

namespace config
{

using SensorValueIntf = phosphor::modbus::rtu::profile::SensorValueIntf;
using SensorFormat = phosphor::modbus::rtu::profile::SensorFormat;
using SensorRegister = phosphor::modbus::rtu::profile::SensorRegister;
using StatusType = phosphor::modbus::rtu::profile::StatusType;
using StatusBit = phosphor::modbus::rtu::profile::StatusBit;
using FirmwareRegisterType =
    phosphor::modbus::rtu::profile::FirmwareRegisterType;
using FirmwareRegister = phosphor::modbus::rtu::profile::FirmwareRegister;

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
    sdbusplus::object_path inventoryPath;
    sensor_registers_t sensorRegisters;
    status_registers_t statusRegisters;
    firmware_registers_t firmwareRegisters;
};

auto updateBaseConfig(sdbusplus::async::context& ctx,
                      const sdbusplus::object_path& objectPath,
                      const std::string& interfaceName, Config& config)
    -> sdbusplus::async::task<bool>;

} // namespace config

} // namespace phosphor::modbus::rtu::device
