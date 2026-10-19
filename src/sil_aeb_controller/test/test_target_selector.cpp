// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "sil_aeb_controller/aeb_fsm.hpp"

namespace sil_aeb_controller
{
namespace
{

AebObstacle MakeObstacle(
  std::uint32_t id, double x, double y, double vx, double closing_speed)
{
  AebObstacle obstacle;
  obstacle.track_id = id;
  obstacle.x = x;
  obstacle.y = y;
  obstacle.vx = vx;
  obstacle.vy = 0.0;
  obstacle.closing_speed = closing_speed;
  obstacle.range = std::hypot(x, y);
  return obstacle;
}

TEST(TargetSelectorTest, ReturnsMinusOneWithNoObstacles)
{
  TargetSelector selector;
  EXPECT_EQ(-1, selector.Select({}, 20.0));
}

TEST(TargetSelectorTest, SelectsTheSmallestTtc)
{
  TargetSelector selector;
  const std::vector<AebObstacle> obstacles{
    MakeObstacle(1, 60.0, 0.0, -10.0, 10.0),  // TTC 6.0 s
    MakeObstacle(2, 30.0, 0.0, -10.0, 10.0),  // TTC 3.0 s
    MakeObstacle(3, 45.0, 0.0, -10.0, 10.0),  // TTC 4.5 s
  };

  EXPECT_EQ(1, selector.Select(obstacles, 20.0));
}

TEST(TargetSelectorTest, IgnoresObstaclesOutsideTheLane)
{
  TargetSelector selector;
  const std::vector<AebObstacle> obstacles{
    MakeObstacle(1, 30.0, 5.0, -10.0, 10.0),  // 5 m to the left: not in lane
  };

  EXPECT_EQ(-1, selector.Select(obstacles, 20.0));
}

TEST(TargetSelectorTest, IgnoresObstaclesBehindTheEgo)
{
  TargetSelector selector;
  const std::vector<AebObstacle> obstacles{
    MakeObstacle(1, -20.0, 0.0, 10.0, 10.0),
  };

  EXPECT_EQ(-1, selector.Select(obstacles, 20.0));
}

TEST(TargetSelectorTest, IgnoresObstaclesBeyondDetectionRange)
{
  TargetSelector selector;
  const std::vector<AebObstacle> obstacles{
    MakeObstacle(1, 200.0, 0.0, -10.0, 10.0),
  };

  EXPECT_EQ(-1, selector.Select(obstacles, 20.0));
}

TEST(TargetSelectorTest, IgnoresNonClosingObstacles)
{
  TargetSelector selector;
  AebObstacle receding = MakeObstacle(1, 30.0, 0.0, 5.0, 0.0);
  // Positive vx means the target moves away along the ego x-axis, so the
  // radial projection gives a negative closing speed.
  EXPECT_EQ(-1, selector.Select({receding}, 0.0));
}

TEST(TargetSelectorTest, DerivesClosingSpeedFromRelativeVelocity)
{
  TargetSelector selector;
  AebObstacle obstacle = MakeObstacle(1, 30.0, 0.0, -8.0, 0.0);
  // closing_speed is unset, so it must come from -(x*vx + y*vy) / range,
  // giving an 8 m/s approach and therefore a 30 / 8 s TTC.
  EXPECT_NEAR(30.0 / 8.0, selector.ComputeTtc(obstacle, 0.0), 1e-9);
}

TEST(TargetSelectorTest, FallsBackToEgoSpeedForStationaryTarget)
{
  TargetSelector selector;
  AebObstacle stationary = MakeObstacle(1, 40.0, 0.0, 0.0, 0.0);
  // Ego closing on a stationary target at 20 m/s gives a 2 s TTC.
  EXPECT_NEAR(2.0, selector.ComputeTtc(stationary, 20.0), 1e-9);
}

TEST(TargetSelectorTest, BreaksTtcTiesOnRange)
{
  TargetSelector selector;
  const std::vector<AebObstacle> obstacles{
    MakeObstacle(1, 50.0, 0.0, -10.0, 10.0),
    MakeObstacle(2, 20.0, 0.0, -4.0, 4.0),  // Same 5 s TTC, shorter range.
  };

  EXPECT_EQ(1, selector.Select(obstacles, 20.0));
}

TEST(TargetSelectorTest, LaneBoundaryIsInclusive)
{
  TargetSelector selector;
  const std::vector<AebObstacle> obstacles{
    MakeObstacle(1, 30.0, 2.0, -10.0, 10.0),  // Exactly at the 2 m boundary.
  };

  EXPECT_EQ(0, selector.Select(obstacles, 20.0));
}

TEST(TargetSelectorTest, SelectsTheNearestOfSeveralClosingTargets)
{
  TargetSelector selector;
  const std::vector<AebObstacle> obstacles{
    MakeObstacle(1, 70.0, 1.5, -12.0, 12.0),
    MakeObstacle(2, 12.0, -0.5, -6.0, 6.0),
    MakeObstacle(3, 40.0, 0.0, -9.0, 9.0),
  };

  // TTCs are ~5.8 s, 2.0 s and ~4.4 s, so track 2 is the threat.
  EXPECT_EQ(1, selector.Select(obstacles, 20.0));
}

}  // namespace
}  // namespace sil_aeb_controller
