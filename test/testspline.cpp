#include <gtest/gtest.h>

#include "hermite/spline/spline_impl.hpp"
#include "hermite/spline/spline_vec.hpp"

using namespace hermite;

TEST(SplineImpl, PosTest) {
  double t[4] = {0, 2, 5, 8};
  double y[4] = {1, 2, 0, 0};
  int n = 4;
  double yp1 = 2;
  double ypn = 1;

  double ydd[4];

  spline(t, y, n, yp1, ypn, ydd);

  double y1, y2, y3, y4;
  EXPECT_TRUE(splpos(t, y, ydd, n, 1, &y1));
  EXPECT_TRUE(splpos(t, y, ydd, n, 4, &y2));
  EXPECT_TRUE(splpos(t, y, ydd, n, 5, &y3));
  EXPECT_TRUE(splpos(t, y, ydd, n, 7.5, &y4));

  EXPECT_NEAR(y1, 2.105, 0.01);
  EXPECT_NEAR(y2, 0.712, 0.01);
  EXPECT_NEAR(y3, 0, 0.01);
  EXPECT_NEAR(y4, -0.392, 0.01);
}

TEST(SplineImpl, VelTest) {
  double t[4] = {0, 2, 5, 8};
  double y[4] = {1, 2, 0, 0};
  int n = 4;
  double yp1 = 2;
  double ypn = 1;

  double ydd[4];

  spline(t, y, n, yp1, ypn, ydd);

  double y1, y2, y3, y4;
  EXPECT_TRUE(splvel(t, y, ydd, n, 1, &y1));
  EXPECT_TRUE(splvel(t, y, ydd, n, 4, &y2));
  EXPECT_TRUE(splvel(t, y, ydd, n, 5, &y3));
  EXPECT_TRUE(splvel(t, y, ydd, n, 7.5, &y4));

  EXPECT_NEAR(y1, 0.355, 0.01);
  EXPECT_NEAR(y2, -0.749, 0.01);
  EXPECT_NEAR(y3, -0.644, 0.01);
  EXPECT_NEAR(y4, 0.578, 0.01);
}

TEST(SplineImpl, AccTest) {
  double t[4] = {0, 2, 5, 8};
  double y[4] = {1, 2, 0, 0};
  int n = 4;
  double yp1 = 2;
  double ypn = 1;

  double ydd[4];

  spline(t, y, n, yp1, ypn, ydd);

  double y1, y2, y3, y4;
  EXPECT_TRUE(splacc(t, y, ydd, n, 1, &y1));
  EXPECT_TRUE(splacc(t, y, ydd, n, 4, &y2));
  EXPECT_TRUE(splacc(t, y, ydd, n, 5, &y3));
  EXPECT_TRUE(splacc(t, y, ydd, n, 7.5, &y4));

  EXPECT_NEAR(y1, -1.211, 0.01);
  EXPECT_NEAR(y2, 0.015, 0.01);
  EXPECT_NEAR(y3, 0.193, 0.01);
  EXPECT_NEAR(y4, 0.785, 0.01);
}

TEST(SplineVec, PosTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  auto dd = spline(poses);

  auto y1 = splpos(poses, dd, 1);
  auto y2 = splpos(poses, dd, 4);
  auto y3 = splpos(poses, dd, 5);
  auto y4 = splpos(poses, dd, 7.5);

  EXPECT_NEAR(y1[0], 2.105, 0.01);
  EXPECT_NEAR(y2[0], 0.712, 0.01);
  EXPECT_NEAR(y3[0], 0, 0.01);
  EXPECT_NEAR(y4[0], -0.392, 0.01);
}

TEST(SplineVec, VelTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  auto dd = spline(poses);

  auto y1 = splvel(poses, dd, 1);
  auto y2 = splvel(poses, dd, 4);
  auto y3 = splvel(poses, dd, 5);
  auto y4 = splvel(poses, dd, 7.5);

  EXPECT_NEAR(y1[0], 0.355, 0.01);
  EXPECT_NEAR(y2[0], -0.749, 0.01);
  EXPECT_NEAR(y3[0], -0.644, 0.01);
  EXPECT_NEAR(y4[0], 0.578, 0.01);
}

TEST(SplineVec, AccTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  auto dd = spline(poses);

  auto y1 = splacc(poses, dd, 1);
  auto y2 = splacc(poses, dd, 4);
  auto y3 = splacc(poses, dd, 5);
  auto y4 = splacc(poses, dd, 7.5);

  EXPECT_NEAR(y1[0], -1.211, 0.01);
  EXPECT_NEAR(y2[0], 0.015, 0.01);
  EXPECT_NEAR(y3[0], 0.193, 0.01);
  EXPECT_NEAR(y4[0], 0.785, 0.01);
}
