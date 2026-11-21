#pragma once

#include "domain/track_state.hpp"

#include <gtest/gtest.h>

namespace target::test_support
{

inline void expectTrackStateEquals(const TrackState& actual, const TrackState& expected)
{
    EXPECT_EQ(actual.sphericalPosition().radius.m(), expected.sphericalPosition().radius.m());
    EXPECT_EQ(actual.sphericalPosition().direction.azimuth.rad(), expected.sphericalPosition().direction.azimuth.rad());
    EXPECT_EQ(actual.sphericalPosition().direction.elevation.rad(), expected.sphericalPosition().direction.elevation.rad());
    EXPECT_EQ(actual.dopplerVelocity().mps(), expected.dopplerVelocity().mps());
    EXPECT_EQ(actual.power_db(), expected.power_db());
    EXPECT_EQ(actual.timestamp(), expected.timestamp());
    EXPECT_EQ(actual.position().x.m(), expected.position().x.m());
    EXPECT_EQ(actual.position().y.m(), expected.position().y.m());
    EXPECT_EQ(actual.position().z.m(), expected.position().z.m());
    EXPECT_EQ(actual.velocity().x.mps(), expected.velocity().x.mps());
    EXPECT_EQ(actual.velocity().y.mps(), expected.velocity().y.mps());
    EXPECT_EQ(actual.velocity().z.mps(), expected.velocity().z.mps());
    EXPECT_EQ(actual.acceleration().x.mps2(), expected.acceleration().x.mps2());
    EXPECT_EQ(actual.acceleration().y.mps2(), expected.acceleration().y.mps2());
    EXPECT_EQ(actual.acceleration().z.mps2(), expected.acceleration().z.mps2());
}

} // namespace target::test_support
