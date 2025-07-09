#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace phosphor::modbus::rtu
{

class Message
{
  public:
    static constexpr auto maxADUSize = 256;
    std::array<uint8_t, maxADUSize> raw;
    size_t len = 0;

    // Push to the end of raw message
    Message& operator<<(uint8_t d);
    Message& operator<<(uint16_t d);
    Message& operator<<(uint32_t d);

    // Pop from the end of raw message
    Message& operator>>(uint8_t& d);
    Message& operator>>(uint16_t& d);
    Message& operator>>(uint32_t& d);

    Message() = default;
    Message(const Message&) = delete;
    Message& operator=(const Message&) = delete;

    uint8_t& address = raw[0];
    uint8_t& functionCode = raw[1];

  protected:
    auto appendCRC() -> void;
    auto validate() -> void;
    auto verifyValue(const std::string& name, uint32_t currentValue,
                     uint32_t expectedValue) -> void;

    template <typename T>
    Message& operator>>(std::vector<T>& d)
    {
        for (auto it = d.rbegin(); it != d.rend(); it++)
        {
            *this >> *it;
        }
        return *this;
    }

  private:
    auto generateCRC() -> uint16_t;

    constexpr auto begin() noexcept
    {
        return raw.begin();
    }
    auto end() noexcept
    {
        return raw.begin() + len;
    }
};

} // namespace phosphor::modbus::rtu
