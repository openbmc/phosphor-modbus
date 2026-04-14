#include "common/register_span.hpp"

#include <gtest/gtest.h>

using namespace phosphor::modbus;

// Single register produces a single span.
TEST(RegisterSpanTest, SingleRegister)
{
    std::vector<RegisterInfo> regs = {{.offset = 100, .size = 4}};
    auto spans = buildRegisterSpans(regs, 120);

    ASSERT_EQ(spans.size(), 1);
    EXPECT_EQ(spans[0].startOffset, 100);
    EXPECT_EQ(spans[0].totalSize, 4);
    ASSERT_EQ(spans[0].registerIndices.size(), 1);
    EXPECT_EQ(spans[0].registerIndices[0], 0);
}

// Contiguous registers are merged into one span.
TEST(RegisterSpanTest, ContiguousRegistersMerge)
{
    std::vector<RegisterInfo> regs = {
        {.offset = 100, .size = 4},
        {.offset = 104, .size = 3},
        {.offset = 107, .size = 2},
    };
    auto spans = buildRegisterSpans(regs, 120);

    ASSERT_EQ(spans.size(), 1);
    EXPECT_EQ(spans[0].startOffset, 100);
    EXPECT_EQ(spans[0].totalSize, 9);
    EXPECT_EQ(spans[0].registerIndices.size(), 3);
}

// Non-contiguous registers produce separate spans.
TEST(RegisterSpanTest, NonContiguousRegistersSplit)
{
    std::vector<RegisterInfo> regs = {
        {.offset = 100, .size = 4},
        {.offset = 106, .size = 3}, // gap = 2
    };
    auto spans = buildRegisterSpans(regs, 120);

    ASSERT_EQ(spans.size(), 2);
    EXPECT_EQ(spans[0].startOffset, 100);
    EXPECT_EQ(spans[0].totalSize, 4);
    EXPECT_EQ(spans[1].startOffset, 106);
    EXPECT_EQ(spans[1].totalSize, 3);
}

// Span is split when maxSpanLength would be exceeded.
TEST(RegisterSpanTest, MaxSpanLengthEnforced)
{
    std::vector<RegisterInfo> regs = {
        {.offset = 100, .size = 8},
        {.offset = 108, .size = 8}, // contiguous, total=16 > maxSpanLength=12
    };
    auto spans = buildRegisterSpans(regs, 12);

    ASSERT_EQ(spans.size(), 2);
    EXPECT_EQ(spans[0].startOffset, 100);
    EXPECT_EQ(spans[0].totalSize, 8);
    EXPECT_EQ(spans[1].startOffset, 108);
    EXPECT_EQ(spans[1].totalSize, 8);
}

// Unsorted input registers are handled correctly.
TEST(RegisterSpanTest, UnsortedInputSorted)
{
    std::vector<RegisterInfo> regs = {
        {.offset = 110, .size = 2},
        {.offset = 100, .size = 4},
        {.offset = 104, .size = 3},
    };
    auto spans = buildRegisterSpans(regs, 120);

    ASSERT_EQ(spans.size(), 1);
    EXPECT_EQ(spans[0].startOffset, 100);
    EXPECT_EQ(spans[0].totalSize, 12); // 110 + 2 - 100
    // Indices refer to original positions
    EXPECT_EQ(spans[0].registerIndices[0], 1); // offset 100
    EXPECT_EQ(spans[0].registerIndices[1], 2); // offset 104
    EXPECT_EQ(spans[0].registerIndices[2], 0); // offset 110
}

// Empty input produces no spans.
TEST(RegisterSpanTest, EmptyInput)
{
    std::vector<RegisterInfo> regs = {};
    auto spans = buildRegisterSpans(regs, 120);
    EXPECT_TRUE(spans.empty());
}
