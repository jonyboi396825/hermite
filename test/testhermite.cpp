#include <gtest/gtest.h>

#include "hermite/hermite.hpp"

using namespace hermite;

TEST(Hermite, InsertTest) {
  Hermite<1> h;
  Pose<1> p1{-2, {0}, {0}};
  Pose<1> p2{8, {0}, {0}};

  h.insert(p1);
  h.insert(p2);

  EXPECT_NEAR(h.getLowestTime(), -2, 0.001);
  EXPECT_NEAR(h.getHighestTime(), 8, 0.001);
}

TEST(Hermite, InsertAlreadyExistsTest) {
  Hermite<1> h;
  Pose<1> p1{-2, {0}, {0}};
  Pose<1> p2{-2, {0}, {0}};

  h.insert(p1);
  h.insert(p2);

  EXPECT_NEAR(h.getLowestTime(), -2, 0.001);
  EXPECT_NEAR(h.getHighestTime(), -2, 0.001);
}

TEST(Hermite, ReplaceTest) {
  Hermite<1> h;
  Pose<1> p1{-2, {0}, {0}};
  Pose<1> p2{-2, {1}, {4}};

  h.insert(p1);
  h.replace(p2);

  auto res = h.getAllWaypoints();
  EXPECT_EQ(res.size(), 1);
  EXPECT_NEAR(res[0].getTime(), -2, 0.001);
  EXPECT_NEAR(res[0].getPos()[0], 1, 0.001);
  EXPECT_NEAR(res[0].getVel()[0], 4, 0.001);
}

TEST(Hermite, ReplaceNoExistTest) {
  Hermite<1> h;
  Pose<1> p1{-2, {0}, {0}};
  Pose<1> p2{-6, {1}, {4}};

  h.insert(p1);
  h.replace(p2);

  auto res = h.getAllWaypoints();
  EXPECT_EQ(res.size(), 1);
  EXPECT_NEAR(res[0].getTime(), -2, 0.001);
  EXPECT_NEAR(res[0].getPos()[0], 0, 0.001);
  EXPECT_NEAR(res[0].getVel()[0], 0, 0.001);
}

TEST(Hermite, InsertOrReplace1) {
  Hermite<1> h;
  Pose<1> p1{-2, {0}, {0}};
  Pose<1> p2{-2, {1}, {4}};

  h.insertOrReplace(p1);
  h.insertOrReplace(p2);

  auto res = h.getAllWaypoints();
  EXPECT_EQ(res.size(), 1);
  EXPECT_NEAR(res[0].getTime(), -2, 0.001);
  EXPECT_NEAR(res[0].getPos()[0], 1, 0.001);
  EXPECT_NEAR(res[0].getVel()[0], 4, 0.001);
}

TEST(Hermite, InsertOrReplace2) {
  Hermite<1> h;
  Pose<1> p1{-2, {0}, {0}};
  Pose<1> p2{6, {1}, {4}};

  h.insertOrReplace(p2);
  h.insertOrReplace(p1);

  auto res = h.getAllWaypoints();
  EXPECT_EQ(res.size(), 2);
  EXPECT_NEAR(res[0].getTime(), -2, 0.001);
  EXPECT_NEAR(res[0].getPos()[0], 0, 0.001);
  EXPECT_NEAR(res[0].getVel()[0], 0, 0.001);
  EXPECT_NEAR(res[1].getTime(), 6, 0.001);
  EXPECT_NEAR(res[1].getPos()[0], 1, 0.001);
  EXPECT_NEAR(res[1].getVel()[0], 4, 0.001);
}

TEST(Hermite, EraseWaypointTest) {
  Hermite<1> h;
  Pose<1> p1{-2, {0}, {0}};
  Pose<1> p2{8, {0}, {0}};
  Pose<1> p3{10, {0}, {0}};

  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  Pose<1> er1{8, {0}, {0}};
  Pose<1> er2{10, {3}, {-5}};

  h.erase(er1);
  h.erase(er2);

  EXPECT_NEAR(h.getLowestTime(), -2, 0.001);
  EXPECT_NEAR(h.getHighestTime(), -2, 0.001);
}

TEST(Hermite, EraseTimeTest) {
  Hermite<1> h;
  Pose<1> p1{-2, {0}, {0}};
  Pose<1> p2{8, {0}, {0}};
  Pose<1> p3{10, {0}, {0}};

  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  h.erase(8);
  h.erase(10);

  EXPECT_NEAR(h.getLowestTime(), -2, 0.001);
  EXPECT_NEAR(h.getHighestTime(), -2, 0.001);
}

TEST(Hermite, AllWaypointTest) {
  Hermite<1> h;
  Pose<1> p1{-2, {3}, {1}};
  Pose<1> p2{8, {-2}, {4}};
  Pose<1> p3{10, {1}, {2}};

  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

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

TEST(Hermite, PosEmptyTest) {
  Hermite<2> h;
  auto res = h.getPos(4);
  EXPECT_TRUE(res.isZero());
}

TEST(Hermite, VelEmptyTest) {
  Hermite<2> h;
  auto res = h.getVel(4);
  EXPECT_TRUE(res.isZero());
}

TEST(Hermite, AccEmptyTest) {
  Hermite<2> h;
  auto res = h.getAcc(4);
  EXPECT_TRUE(res.isZero());
}

TEST(Hermite, PosNotEnoughTest) {
  Hermite<2> h;

  Pose<2> p1{-2, {3}, {1}};
  h.insert(p1);

  auto res = h.getPos(4);
  EXPECT_TRUE(res.isZero());
}

TEST(Hermite, VelNotEnoughTest) {
  Hermite<2> h;

  Pose<2> p1{-2, {3}, {1}};
  h.insert(p1);

  auto res = h.getVel(4);
  EXPECT_TRUE(res.isZero());
}

TEST(Hermite, AccNotEnoughTest) {
  Hermite<2> h;

  Pose<2> p1{-2, {3}, {1}};
  h.insert(p1);

  auto res = h.getAcc(4);
  EXPECT_TRUE(res.isZero());
}

TEST(Hermite, PosTest2Waypoints) {
  Hermite<1> h;

  Pose<1> begin{-6, {-7}, {5}};
  Pose<1> end{8, {2}, {1}};

  h.insert(end);
  h.insert(begin);

  EXPECT_NEAR(h.getPos(-3)[0], 2.818, 0.01);
  EXPECT_NEAR(h(-2.4)[0], 3.725, 0.01); // testing operator()
  EXPECT_NEAR(h.getPos(-1.1)[0], 4.772, 0.01);
  EXPECT_NEAR(h.getPos(0)[0], 4.869, 0.01);
  EXPECT_NEAR(h.getPos(1)[0], 4.5, 0.01);
}

TEST(Hermite, VelTest2Waypoints) {
  Hermite<1> h;

  Pose<1> begin{-6, {-7}, {5}};
  Pose<1> end{8, {2}, {1}};

  h.insert(end);
  h.insert(begin);

  EXPECT_NEAR(h.getVel(-3)[0], 1.762, 0.01);
  EXPECT_NEAR(h.getVel(-2.4)[0], 1.270, 0.01);
  EXPECT_NEAR(h.getVel(-1.1)[0], 0.382, 0.01);
  EXPECT_NEAR(h.getVel(0)[0], -0.178, 0.01);
  EXPECT_NEAR(h.getVel(1)[0], -0.536, 0.01);
}

TEST(Hermite, AccTest2Waypoints) {
  Hermite<1> h;

  Pose<1> begin{-6, {-7}, {5}};
  Pose<1> end{8, {2}, {1}};

  h.insert(end);
  h.insert(begin);

  EXPECT_NEAR(h.getAcc(-3)[0], -0.863, 0.01);
  EXPECT_NEAR(h.getAcc(-2.4)[0], -0.776, 0.01);
  EXPECT_NEAR(h.getAcc(-1.1)[0], -0.589, 0.01);
  EXPECT_NEAR(h.getAcc(0)[0], -0.430, 0.01);
  EXPECT_NEAR(h.getAcc(1)[0], -0.286, 0.01);
}

TEST(Hermite, PosTooHighTest2Points) {
  Hermite<1> h;

  Pose<1> begin{-6, {-7}, {5}};
  Pose<1> end{8, {2}, {1}};

  h.insert(end);
  h.insert(begin);

  EXPECT_NEAR(h.getPos(10)[0], 5.641, 0.01);
  EXPECT_NEAR(h.getPos(12)[0], 13.335, 0.01);
}

TEST(Hermite, VelTooHighTest2Points) {
  Hermite<1> h;

  Pose<1> begin{-6, {-7}, {5}};
  Pose<1> end{8, {2}, {1}};

  h.insert(end);
  h.insert(begin);

  EXPECT_NEAR(h.getVel(11.5)[0], 4.420, 0.01);
  EXPECT_NEAR(h.getVel(12.3)[0], 5.449, 0.01);
}

TEST(Hermite, AccTooHighTest2Points) {
  Hermite<1> h;

  Pose<1> begin{-2, {-7}, {5}};
  Pose<1> end{6, {2}, {1}};

  h.insert(end);
  h.insert(begin);

  EXPECT_NEAR(h.getAcc(8)[0], 1.609, 0.01);
  EXPECT_NEAR(h.getAcc(12.3)[0], 3.121, 0.01);
}

TEST(Hermite, PosTooLowTest2Points) {
  Hermite<1> h;

  Pose<1> begin{-2, {-7}, {5}};
  Pose<1> end{6, {2}, {1}};

  h.insert(end);
  h.insert(begin);

  EXPECT_NEAR(h.getPos(-4)[0], -21.281, 0.01);
  EXPECT_NEAR(h.getPos(-2.2)[0], -8.039, 0.01);
}

TEST(Hermite, VelTooLowTest2Points) {
  Hermite<1> h;

  Pose<1> begin{-2, {-7}, {5}};
  Pose<1> end{6, {2}, {1}};

  h.insert(end);
  h.insert(begin);

  EXPECT_NEAR(h.getVel(-4)[0], 9.516, 0.01);
  EXPECT_NEAR(h.getVel(-2.2)[0], 5.388, 0.01);
}

TEST(Hermite, AccTooLowTest2Points) {
  Hermite<1> h;

  Pose<1> begin{-2, {-7}, {5}};
  Pose<1> end{6, {2}, {1}};

  h.insert(end);
  h.insert(begin);

  EXPECT_NEAR(h.getAcc(-4)[0], -2.609, 0.01);
  EXPECT_NEAR(h.getAcc(-2.2)[0], -1.977, 0.01);
}

TEST(Hermite, PosTestManyWaypoints) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getPos(-3)[0], -2, 0.01);
  EXPECT_NEAR(h.getPos(-1.5)[0], -0.375, 0.01);
  EXPECT_NEAR(h.getPos(0)[0], 2, 0.01);
  EXPECT_NEAR(h.getPos(1)[0], 2.25, 0.01);
  EXPECT_NEAR(h.getPos(2)[0], 3, 0.01);
  EXPECT_NEAR(h.getPos(3.5)[0], 3.227, 0.01);
  EXPECT_NEAR(h.getPos(6)[0], 0, 0.01);
}

TEST(Hermite, VelTestManyWaypoints) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getVel(-3)[0], 0, 0.01);
  EXPECT_NEAR(h.getVel(-1.5)[0], 1.75, 0.01);
  EXPECT_NEAR(h.getVel(0)[0], 1, 0.01);
  EXPECT_NEAR(h.getVel(1)[0], 0, 0.01);
  EXPECT_NEAR(h.getVel(2)[0], 2, 0.01);
  EXPECT_NEAR(h.getVel(3.5)[0], -1.211, 0.01);
  EXPECT_NEAR(h.getVel(6)[0], 0, 0.01);
}

TEST(Hermite, AccTestManyWaypoints) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getAcc(-3)[0], 2, 0.01);
  EXPECT_NEAR(h.getAcc(-1.5)[0], 0.333, 0.01);
  EXPECT_NEAR(h.getAcc(0)[0], -2.5, 0.01);
  EXPECT_NEAR(h.getAcc(1)[0], 0.5, 0.01);
  EXPECT_NEAR(h.getAcc(2)[0], -3.125, 0.01);
  EXPECT_NEAR(h.getAcc(3.5)[0], -1.156, 0.01);
  EXPECT_NEAR(h.getAcc(6)[0], 2.125, 0.01);
}

TEST(Hermite, PosTooHighTest) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getPos(6.1)[0], 0.011, 0.01);
}

TEST(Hermite, VelTooHighTest) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getVel(6.1)[0], 0.219, 0.01);
}

TEST(Hermite, AccTooHighTest) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getAcc(6.1)[0], 2.256, 0.01);
}

TEST(Hermite, PosTooLowTest) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getPos(-3.1)[0], -1.990, 0.01);
}

TEST(Hermite, VelTooLowTest) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getVel(-3.1)[0], -0.206, 0.01);
}

TEST(Hermite, AccTooLowTest) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getAcc(-3.1)[0], 2.111, 0.01);
}

TEST(Hermite, MaxPosTest) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getMaxDistance(0.001), 3.714, 0.01);
}

TEST(Hermite, MaxSpeedTest) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getMaxSpeed(0.001), 2, 0.01);
}

TEST(Hermite, MaxAccTest) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getMaxAcceleration(0.001), 3.5, 0.01);
}

TEST(Hermite, ArcLengthTest) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  EXPECT_NEAR(h.getLength(0.001), 9.445, 0.01);
}

TEST(Hermite, CopyTest) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  Hermite<1> h2{h};

  EXPECT_NEAR(h2.getPos(-3)[0], -2, 0.01);
  EXPECT_NEAR(h2.getPos(-1.5)[0], -0.375, 0.01);
  EXPECT_NEAR(h2.getPos(0)[0], 2, 0.01);
  EXPECT_NEAR(h2.getPos(1)[0], 2.25, 0.01);
  EXPECT_NEAR(h2.getPos(2)[0], 3, 0.01);
  EXPECT_NEAR(h2.getPos(3.5)[0], 3.227, 0.01);
  EXPECT_NEAR(h2.getPos(6)[0], 0, 0.01);
}

TEST(Hermite, AssignTest) {
  Hermite<1> h;

  Pose<1> p0{-3, {-2}, {0}};
  Pose<1> p1{0, {2}, {1}};
  Pose<1> p2{2, {3}, {2}};
  Pose<1> p3{6, {0}, {0}};

  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  Hermite<1> h2;
  h2.insert({-2, {1}, {5}});
  h2.insert({-4, {5}, {1}});
  h2.insert({2, {6}, {0}});

  h2 = h;

  EXPECT_NEAR(h2.getPos(-3)[0], -2, 0.01);
  EXPECT_NEAR(h2.getPos(-1.5)[0], -0.375, 0.01);
  EXPECT_NEAR(h2.getPos(0)[0], 2, 0.01);
  EXPECT_NEAR(h2.getPos(1)[0], 2.25, 0.01);
  EXPECT_NEAR(h2.getPos(2)[0], 3, 0.01);
  EXPECT_NEAR(h2.getPos(3.5)[0], 3.227, 0.01);
  EXPECT_NEAR(h2.getPos(6)[0], 0, 0.01);
}
