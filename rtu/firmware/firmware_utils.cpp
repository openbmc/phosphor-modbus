#include "firmware_utils.hpp"

namespace phosphor::modbus::rtu::device
{

auto convertRegisterValue(std::span<const uint16_t> registers,
                          const ProfileIntf::FirmwareRegister& reg)
    -> std::string
{
    std::string strValue;

    if (reg.format == ProfileIntf::FirmwareFormat::integer)
    {
        uint64_t intValue = 0;
        for (const auto& value : registers)
        {
            intValue = (intValue << 16) | value;
        }
        strValue = std::to_string(intValue);
    }
    else
    {
        for (const auto& value : registers)
        {
            strValue += static_cast<char>((value >> 8) & 0xFF);
            strValue += static_cast<char>(value & 0xFF);
        }
    }

    return strValue;
}

} // namespace phosphor::modbus::rtu::device
