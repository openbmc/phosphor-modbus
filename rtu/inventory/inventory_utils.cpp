#include "inventory_utils.hpp"

#include <string>
#include <type_traits>
#include <variant>

namespace phosphor::modbus::rtu::inventory
{

auto matchesProbeValue(std::span<const uint16_t> readBuffer,
                       const ProfileIntf::ProbeRegister& probe) -> bool
{
    return std::visit(
        [&readBuffer](const auto& expected) -> bool {
            using T = std::decay_t<decltype(expected)>;
            if constexpr (std::is_same_v<T, uint64_t>)
            {
                uint64_t value = 0;
                for (const auto& reg : readBuffer)
                {
                    value = (value << 16) | reg;
                }
                return value == expected;
            }
            else // std::string
            {
                std::string value;
                for (const auto& reg : readBuffer)
                {
                    value += static_cast<char>((reg >> 8) & 0xFF);
                    value += static_cast<char>(reg & 0xFF);
                }
                // Remove null characters
                std::erase(value, '\0');
                return value == expected;
            }
        },
        probe.expectedValue);
}

auto convertRegisterValue(std::span<const uint16_t> registers,
                          const ProfileIntf::InventoryRegister& reg)
    -> std::string
{
    if (reg.format == ProfileIntf::InventoryFormat::integer)
    {
        uint32_t intValue = 0;
        for (const auto& value : registers)
        {
            intValue = (intValue << 16) | value;
        }
        return std::to_string(intValue);
    }

    std::string strValue;
    for (const auto& value : registers)
    {
        strValue += static_cast<char>((value >> 8) & 0xFF);
        strValue += static_cast<char>(value & 0xFF);
    }
    return strValue;
}

} // namespace phosphor::modbus::rtu::inventory
