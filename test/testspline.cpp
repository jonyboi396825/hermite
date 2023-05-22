#include <gtest/gtest.h>

#include "hermite/spline/spline_impl.hpp"

using namespace hermite;

TEST(SplineImpl, PosTest) {
  double t[5] = {0, 0, 2, 5, 8};
  double y[5] = {0, 1, 2, 0, 0};
  int n = 4;
  double yp1 = 2;
  double ypn = 1;

  double ydd[5];

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
  double t[5] = {0, 0, 2, 5, 8};
  double y[5] = {0, 1, 2, 0, 0};
  int n = 4;
  double yp1 = 2;
  double ypn = 1;

  double ydd[5];

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
  double t[5] = {0, 0, 2, 5, 8};
  double y[5] = {0, 1, 2, 0, 0};
  int n = 4;
  double yp1 = 2;
  double ypn = 1;

  double ydd[5];

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
