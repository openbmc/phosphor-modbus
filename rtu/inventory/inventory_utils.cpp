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

} // namespace phosphor::modbus::rtu::inventory
