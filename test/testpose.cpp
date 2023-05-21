#include <gtest/gtest.h>

#include "hermite/pose.hpp"

using namespace hermite;

TEST(Pose, Default) {
  Pose<1> pose;
  EXPECT_NEAR(pose.getTime(), 0, 0.000001);
  EXPECT_TRUE(pose.getPos().isZero());
  EXPECT_TRUE(pose.getVel().isZero());
}

TEST(Pose, GetPose) {
  Pose<1> pose{0.1, {3}, {0}};
  EXPECT_NEAR(pose.getTime(), 0.1, 0.000001);
  EXPECT_NEAR(pose.getPos()[0], 3, 0.000001);
  EXPECT_NEAR(pose.getVel()[0], 0, 0.000001);
}

TEST(Pose, SetPose) {
  Pose<1> pose;

  pose.setTime(0.1);
  pose.setPos({3});
  pose.setVel({0});

  EXPECT_NEAR(pose.getTime(), 0.1, 0.000001);
  EXPECT_NEAR(pose.getPos()[0], 3, 0.000001);
  EXPECT_NEAR(pose.getVel()[0], 0, 0.000001);
}

TEST(Pose, CopyPose) {
  Pose<1> pose;

  pose.setTime(0.1);
  pose.setPos({3});
  pose.setVel({0});

  Pose<1> pose2{pose};

  EXPECT_NEAR(pose2.getTime(), 0.1, 0.000001);
  EXPECT_NEAR(pose2.getPos()[0], 3, 0.000001);
  EXPECT_NEAR(pose2.getVel()[0], 0, 0.000001);
}

TEST(Pose, AssignPose) {
  Pose<1> pose;

  pose.setTime(0.1);
  pose.setPos({3});
  pose.setVel({0});

  Pose<1> pose2{0, {0.4}, {1.1}};
  pose2 = pose;

  EXPECT_NEAR(pose2.getTime(), 0.1, 0.000001);
  EXPECT_NEAR(pose2.getPos()[0], 3, 0.000001);
  EXPECT_NEAR(pose2.getVel()[0], 0, 0.000001);
}
