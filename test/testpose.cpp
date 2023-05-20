#include <gtest/gtest.h>

#include "hermite/pose.hpp"

using namespace hermite;

TEST(Pose, MakePos) {
  auto pose = makePose<1>(0.1, {3});
  EXPECT_NEAR(pose.time, 0.1, 0.000001);
  EXPECT_NEAR(pose.pos[0], 3, 0.000001);
  EXPECT_NEAR(pose.vel[0], 0, 0.000001);
  EXPECT_NEAR(pose.acc[0], 0, 0.000001);
}

TEST(Pose, MakeVel) {
  auto pose = makePose<1>(0.1, {3}, {-1});
  EXPECT_NEAR(pose.time, 0.1, 0.000001);
  EXPECT_NEAR(pose.pos[0], 3, 0.000001);
  EXPECT_NEAR(pose.vel[0], -1, 0.000001);
  EXPECT_NEAR(pose.acc[0], 0, 0.000001);
}

TEST(Pose, MakeAcc) {
  auto pose = makePose<1>(0.1, {4}, {-2}, {4.5});
  EXPECT_NEAR(pose.time, 0.1, 0.000001);
  EXPECT_NEAR(pose.pos[0], 4, 0.000001);
  EXPECT_NEAR(pose.vel[0], -2, 0.000001);
  EXPECT_NEAR(pose.acc[0], 4.5, 0.000001);
}
