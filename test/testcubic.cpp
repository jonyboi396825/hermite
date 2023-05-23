#include <gtest/gtest.h>

#include "hermite/cubic/cubic_impl.hpp"
#include "hermite/cubic/cubic_vec.hpp"

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
  SplineVec<1> spl{poses};

  auto y1 = spl.splpos(1);
  auto y2 = spl.splpos(4);
  auto y3 = spl.splpos(5);
  auto y4 = spl.splpos(7.5);

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
  SplineVec<1> spl{poses};

  auto y1 = spl.splvel(1);
  auto y2 = spl.splvel(4);
  auto y3 = spl.splvel(5);
  auto y4 = spl.splvel(7.5);

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
  SplineVec<1> spl{poses};

  auto y1 = spl.splacc(1);
  auto y2 = spl.splacc(4);
  auto y3 = spl.splacc(5);
  auto y4 = spl.splacc(7.5);

  EXPECT_NEAR(y1[0], -1.211, 0.01);
  EXPECT_NEAR(y2[0], 0.015, 0.01);
  EXPECT_NEAR(y3[0], 0.193, 0.01);
  EXPECT_NEAR(y4[0], 0.785, 0.01);
}

TEST(SplineVec, CopyTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  SplineVec<1> spl{poses};
  SplineVec<1> spl2{spl};

  auto y1 = spl.splacc(1);
  auto y2 = spl.splacc(4);
  auto y3 = spl.splacc(5);
  auto y4 = spl.splacc(7.5);

  EXPECT_NEAR(y1[0], -1.211, 0.01);
  EXPECT_NEAR(y2[0], 0.015, 0.01);
  EXPECT_NEAR(y3[0], 0.193, 0.01);
  EXPECT_NEAR(y4[0], 0.785, 0.01);
}

TEST(SplineVec, AssignTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  Pose<1> p5{1, {10}, {2.2}};
  Pose<1> p6{2.5, {24}, {0.1}};
  Pose<1> p7{5.2, {0.11}, {2.3}};
  Pose<1> p8{80, {0.03}, {1.45}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  std::vector<Pose<1>> poses2{p5, p6, p7, p8};
  SplineVec<1> spl{poses};
  SplineVec<1> spl2{poses2};

  spl2 = spl;

  auto y1 = spl.splacc(1);
  auto y2 = spl.splacc(4);
  auto y3 = spl.splacc(5);
  auto y4 = spl.splacc(7.5);

  EXPECT_NEAR(y1[0], -1.211, 0.01);
  EXPECT_NEAR(y2[0], 0.015, 0.01);
  EXPECT_NEAR(y3[0], 0.193, 0.01);
  EXPECT_NEAR(y4[0], 0.785, 0.01);
}
