#include "register_span.hpp"

#include <algorithm>
#include <numeric>

namespace phosphor::modbus
{

auto buildRegisterSpans(const std::vector<RegisterInfo>& registers,
                        uint16_t maxSpanLength, uint16_t maxGap)
    -> std::vector<RegisterSpan>
{
    if (registers.empty())
    {
        return {};
    }

    // Build index list sorted by offset
    std::vector<size_t> sortedIndices(registers.size());
    std::iota(sortedIndices.begin(), sortedIndices.end(), 0);
    std::sort(sortedIndices.begin(), sortedIndices.end(),
              [&registers](size_t a, size_t b) {
                  return registers[a].offset < registers[b].offset;
              });

    std::vector<RegisterSpan> spans;
    auto firstIdx = sortedIndices[0];
    const auto& first = registers[firstIdx];
    spans.push_back({.startOffset = first.offset,
                     .totalSize = first.size,
                     .registerIndices = {firstIdx}});

    for (size_t i = 1; i < sortedIndices.size(); i++)
    {
        auto idx = sortedIndices[i];
        const auto& reg = registers[idx];
        auto& current = spans.back();

        uint16_t spanEnd = current.startOffset + current.totalSize;
        uint16_t gap = (reg.offset > spanEnd) ? (reg.offset - spanEnd) : 0;
        uint16_t newSize = (reg.offset + reg.size) - current.startOffset;

        if (gap <= maxGap && newSize <= maxSpanLength)
        {
            current.totalSize = newSize;
            current.registerIndices.push_back(idx);
        }
        else
        {
            spans.push_back({.startOffset = reg.offset,
                             .totalSize = reg.size,
                             .registerIndices = {idx}});
        }
    }

    return spans;
}

} // namespace phosphor::modbus
