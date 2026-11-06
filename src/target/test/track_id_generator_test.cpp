#include "service/track_id_generator.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace target
{
namespace
{

TEST(TrackIdGeneratorTest, AdvancesOnlyAfterCommit)
{
    TrackIdGenerator generator;
    EXPECT_EQ(generator.nextId(), 1u);
    EXPECT_EQ(generator.nextId(), 1u);
    static_assert(noexcept(generator.commit()));
    generator.commit();
    EXPECT_EQ(generator.nextId(), 2u);
    EXPECT_EQ(generator.nextId(), 2u);
}

TEST(TrackIdGeneratorTest, ExhaustionDoesNotWrapOrReuseIds)
{
    TrackIdGenerator generator;
    const auto maximum = std::numeric_limits<std::uint16_t>::max();
    for (std::uint32_t id = 1; id <= maximum; ++id)
    {
        ASSERT_EQ(generator.nextId(), id);
        generator.commit();
    }
    EXPECT_THROW(generator.nextId(), std::overflow_error);
    generator.commit();
    EXPECT_THROW(generator.nextId(), std::overflow_error);
}

} // namespace
} // namespace target
