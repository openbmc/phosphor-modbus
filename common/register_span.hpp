#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace phosphor::modbus
{

struct RegisterInfo
{
    uint16_t offset;
    uint8_t size;
};

struct RegisterSpan
{
    uint16_t startOffset;
    uint16_t totalSize;
    // Indices into the original register list, preserving caller's ordering.
    std::vector<size_t> registerIndices;
};

/**
 * @brief Build merged spans from a list of registers.
 *
 * Registers are sorted by offset and merged into contiguous spans
 * as long as the resulting span size <= maxSpanLength.
 *
 * @param registers     Input register metadata (offset + size).
 * @param maxSpanLength Upper bound on a single span's total register count.
 * @return Merged spans, each referencing the original register indices.
 */
auto buildRegisterSpans(const std::vector<RegisterInfo>& registers,
                        uint16_t maxSpanLength) -> std::vector<RegisterSpan>;

} // namespace phosphor::modbus
