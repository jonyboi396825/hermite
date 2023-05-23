#include <vector>

#include <gtest/gtest.h>

#include <hermite/cubic.hpp>

using namespace hermite;

TEST(Cubic, AllWaypointTest) {
  Pose<1> p1{-2, {3}, {1}};
  Pose<1> p2{8, {-2}, {4}};
  Pose<1> p3{10, {1}, {2}};

  std::vector<Pose<1>> vec{p1, p2, p3};
  Cubic<1> h{vec};

  auto res = h.getAllWaypoints();
  auto res0 = res[0];
  auto res1 = res[1];
  auto res2 = res[2];

  EXPECT_NEAR(res0.getTime(), -2, 0.001);
  EXPECT_NEAR(res0.getPos()[0], 3, 0.001);
  EXPECT_NEAR(res0.getVel()[0], 1, 0.001);
  EXPECT_NEAR(res1.getTime(), 8, 0.001);
  EXPECT_NEAR(res1.getPos()[0], -2, 0.001);
  EXPECT_NEAR(res1.getVel()[0], 4, 0.001);
  EXPECT_NEAR(res2.getTime(), 10, 0.001);
  EXPECT_NEAR(res2.getPos()[0], 1, 0.001);
  EXPECT_NEAR(res2.getVel()[0], 2, 0.001);
}

TEST(Cubic, PosEmptyTest) {
  Cubic<1> h;
  auto res = h.getPos(4);
  EXPECT_TRUE(res.isZero());
}

TEST(Cubic, VelEmptyTest) {
  Cubic<2> h;
  auto res = h.getVel(4);
  EXPECT_TRUE(res.isZero());
}

TEST(Cubic, AccEmptyTest) {
  Cubic<2> h;
  auto res = h.getAcc(4);
  EXPECT_TRUE(res.isZero());
}

TEST(Cubic, PosNotEnoughTest) {
  Pose<1> p1{-2, {3}, {1}};

  std::vector<Pose<1>> vec{p1};
  Cubic<1> h{vec};

  auto res = h.getPos(4);
  EXPECT_TRUE(res.isZero());
}

TEST(Cubic, VelNotEnoughTest) {
  Pose<1> p1{-2, {3}, {1}};

  std::vector<Pose<1>> vec{p1};
  Cubic<1> h{vec};

  auto res = h.getVel(4);
  EXPECT_TRUE(res.isZero());
}

TEST(Cubic, AccNotEnoughTest) {
  Pose<1> p1{-2, {3}, {1}};

  std::vector<Pose<1>> vec{p1};
  Cubic<1> h{vec};

  auto res = h.getAcc(4);
  EXPECT_TRUE(res.isZero());
}

TEST(Cubic, PosTestManyWaypoints) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  auto y1 = spl.getPos(1);
  auto y2 = spl.getPos(4);
  auto y3 = spl.getPos(5);
  auto y4 = spl.getPos(7.5);

  EXPECT_NEAR(y1[0], 2.105, 0.01);
  EXPECT_NEAR(y2[0], 0.712, 0.01);
  EXPECT_NEAR(y3[0], 0, 0.01);
  EXPECT_NEAR(y4[0], -0.392, 0.01);
}

TEST(Cubic, VelTestManyWaypoints) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  auto y1 = spl.getVel(1);
  auto y2 = spl.getVel(4);
  auto y3 = spl.getVel(5);
  auto y4 = spl.getVel(7.5);

  EXPECT_NEAR(y1[0], 0.355, 0.01);
  EXPECT_NEAR(y2[0], -0.749, 0.01);
  EXPECT_NEAR(y3[0], -0.644, 0.01);
  EXPECT_NEAR(y4[0], 0.578, 0.01);
}

TEST(Cubic, AccTestManyWaypoints) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  auto y1 = spl.getAcc(1);
  auto y2 = spl.getAcc(4);
  auto y3 = spl.getAcc(5);
  auto y4 = spl.getAcc(7.5);

  EXPECT_NEAR(y1[0], -1.211, 0.01);
  EXPECT_NEAR(y2[0], 0.015, 0.01);
  EXPECT_NEAR(y3[0], 0.193, 0.01);
  EXPECT_NEAR(y4[0], 0.785, 0.01);
}

TEST(Cubic, PosTooHighTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  auto res = spl.getPos(8.1);

  EXPECT_NEAR(res[0], 0.105, 0.01);
}

TEST(Cubic, VelTooHighTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  auto res = spl.getVel(8.1);

  EXPECT_NEAR(res[0], 1.092, 0.01);
}

TEST(Cubic, AccTooHighTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  auto res = spl.getAcc(8.1);

  EXPECT_NEAR(res[0], 0.927, 0.01);
}

TEST(Cubic, PosTooLowTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  auto res = spl.getPos(-0.1);

  EXPECT_NEAR(res[0], 0.789, 0.01);
}

TEST(Cubic, VelTooLowTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  auto res = spl.getVel(-0.1);

  EXPECT_NEAR(res[0], 2.212, 0.01);
}

TEST(Cubic, AccTooLowTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  auto res = spl.getAcc(-0.1);

  EXPECT_NEAR(res[0], -2.165, 0.01);
}

TEST(Cubic, MaxPosTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  EXPECT_NEAR(spl.getMaxDistance(0.001), 2.162, 0.01);
}

TEST(Cubic, MaxSpeedTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  EXPECT_NEAR(spl.getMaxSpeed(0.001), 2, 0.01);
}

TEST(Cubic, MaxAccTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  EXPECT_NEAR(spl.getMaxAcceleration(0.001), 2.079, 0.01);
}

TEST(Cubic, ArcLengthTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};

  EXPECT_NEAR(spl.getLength(0.001), 4.571, 0.01);
}

TEST(Cubic, CopyTest) {
  Pose<1> p1{0, {1}, {2}};
  Pose<1> p2{2, {2}, {0}};
  Pose<1> p3{5, {0}, {0}};
  Pose<1> p4{8, {0}, {1}};

  std::vector<Pose<1>> poses{p1, p2, p3, p4};
  Cubic<1> spl{poses};
  Cubic<1> spl2{spl};

  auto y1 = spl.getPos(1);
  auto y2 = spl.getPos(4);
  auto y3 = spl.getPos(5);
  auto y4 = spl.getPos(7.5);

  EXPECT_NEAR(y1[0], 2.105, 0.01);
  EXPECT_NEAR(y2[0], 0.712, 0.01);
  EXPECT_NEAR(y3[0], 0, 0.01);
  EXPECT_NEAR(y4[0], -0.392, 0.01);
}

TEST(Cubic, AssignTest) {
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
  Cubic<1> spl{poses};
  Cubic<1> spl2{poses2};

  spl2 = spl;

  auto y1 = spl.getPos(1);
  auto y2 = spl.getPos(4);
  auto y3 = spl.getPos(5);
  auto y4 = spl.getPos(7.5);

  EXPECT_NEAR(y1[0], 2.105, 0.01);
  EXPECT_NEAR(y2[0], 0.712, 0.01);
  EXPECT_NEAR(y3[0], 0, 0.01);
  EXPECT_NEAR(y4[0], -0.392, 0.01);
}
