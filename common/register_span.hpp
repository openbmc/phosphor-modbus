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
 * Registers are sorted by offset and merged into contiguous spans.
 * Two registers merge into the same span when:
 *   1. The gap between them is <= maxGap
 *   2. The resulting span size <= maxSpanLength
 *
 * @param registers     Input register metadata (offset + size).
 * @param maxSpanLength Upper bound on a single span's total register count.
 * @param maxGap        Maximum gap (in registers) between two registers that
 *                      still allows merging into one span.
 * @return Merged spans, each referencing the original register indices.
 */
auto buildRegisterSpans(const std::vector<RegisterInfo>& registers,
                        uint16_t maxSpanLength, uint16_t maxGap)
    -> std::vector<RegisterSpan>;

} // namespace phosphor::modbus
