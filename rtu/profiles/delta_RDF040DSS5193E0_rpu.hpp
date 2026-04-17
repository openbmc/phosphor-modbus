#pragma once

#include "device_profile.hpp"

namespace phosphor::modbus::rtu::profile
{

inline const DeviceProfile deltaRdf040dss5193e0Rpu = {
    .parity = Parity::none,
    .baudRate = 19200,
    .inventoryRegisters =
        {
            {.name = "Model",
             .type = InventoryDataType::model,
             .offset = 6604,
             .size = 8},
            {.name = "BuildDate",
             .type = InventoryDataType::buildDate,
             .offset = 6612,
             .size = 4},
            {.name = "SerialNumber",
             .type = InventoryDataType::serialNumber,
             .offset = 6616,
             .size = 8},
            {.name = "SparePartNumber",
             .type = InventoryDataType::sparePartNumber,
             .offset = 6652,
             .size = 4},
        },
    .sensorRegisters =
        {
            {.name = "RPU_Coolant_Inlet_Temp_C",
             .type = SensorType::temperature,
             .offset = 36866,
             .size = 1,
             .precision = 0,
             .scale = 0.1,
             .shift = 0.0,
             .isSigned = true,
             .format = SensorFormat::floatingPoint},
            {.name = "RPU_Coolant_Outlet_Temp_C",
             .type = SensorType::temperature,
             .offset = 36865,
             .size = 1,
             .precision = 0,
             .scale = 0.1,
             .shift = 0.0,
             .isSigned = true,
             .format = SensorFormat::floatingPoint},
        },
    .statusRegisters =
        {
            {37377,
             {
                 {.name = "RPU_Coolant_Inlet_Temp_C",
                  .type = StatusType::sensorReadingCritical,
                  .bitPosition = 2,
                  .value = true},
                 {.name = "RPU_Coolant_Outlet_Temp_C",
                  .type = StatusType::sensorReadingCritical,
                  .bitPosition = 1,
                  .value = true},
             }},
            {37376,
             {
                 {.name = "RPU_Coolant_Inlet_Temp_C",
                  .type = StatusType::sensorFailure,
                  .bitPosition = 0,
                  .value = true},
                 {.name = "RPU_Coolant_Outlet_Temp_C",
                  .type = StatusType::sensorFailure,
                  .bitPosition = 1,
                  .value = true},
             }},
        },
    .firmwareRegisters =
        {
            {.name = "RPU_PLC_FW_Revision",
             .type = FirmwareRegisterType::version,
             .offset = 6632,
             .size = 4},
        },
};

} // namespace phosphor::modbus::rtu::profile
