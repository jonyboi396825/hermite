#include <gtest/gtest.h>

#include "hermite/hermite_unit.hpp"

using namespace hermite;

TEST(HermiteUnitTest, PosTest) {
  HermiteUnit<1> h{{0}, {2.5}, {-3.8}, {0}};
  EXPECT_NEAR(h.getPos(0)[0], 0, 0.00001);
  EXPECT_NEAR(h.getPos(0.1)[0], -0.2378, 0.00001);
  EXPECT_NEAR(h.getPos(0.5)[0], 0.775, 0.00001);
  EXPECT_NEAR(h.getPos(0.75)[0], 1.93125, 0.00001);
  EXPECT_NEAR(h.getPos(1)[0], 2.5, 0.00001);
}

TEST(HermiteUnitTest, PosOutOfBoundTest) {
  HermiteUnit<1> h{{0}, {2.6}, {-3.8}, {0}};
  EXPECT_TRUE(h.getPos(-50).isZero());
  EXPECT_TRUE(h.getPos(50).isZero());
}

TEST(HermiteUnitTest, VelTest) {
  HermiteUnit<1> h{{3}, {1.5}, {2.8}, {1}};
  EXPECT_NEAR(h.getVel(0)[0], 2.8, 0.00001);
  EXPECT_NEAR(h.getVel(0.1)[0], 0.784, 0.00001);
  EXPECT_NEAR(h.getVel(0.5)[0], -3.2, 0.00001);
  EXPECT_NEAR(h.getVel(0.75)[0], -2.375, 0.00001);
  EXPECT_NEAR(h.getVel(1)[0], 1, 0.00001);
}

TEST(HermiteUnitTest, VelOutOfBoundTest) {
  HermiteUnit<1> h{{3}, {1.5}, {2.8}, {1}};
  EXPECT_TRUE(h.getVel(-1).isZero());
  EXPECT_TRUE(h.getVel(2).isZero());
}

TEST(HermiteUnitTest, AccTest) {
  HermiteUnit<1> h{{1}, {-0.5}, {0}, {4}};
  EXPECT_NEAR(h.getAcc(0)[0], -17, 0.00001);
  EXPECT_NEAR(h.getAcc(0.1)[0], -12.8, 0.00001);
  EXPECT_NEAR(h.getAcc(0.5)[0], 4, 0.00001);
  EXPECT_NEAR(h.getAcc(0.75)[0], 14.5, 0.00001);
  EXPECT_NEAR(h.getAcc(1)[0], 25, 0.00001);
}

TEST(HermiteUnitTest, AccOutOfBoundTest) {
  HermiteUnit<1> h{{1}, {-0.5}, {0}, {4}};
  EXPECT_TRUE(h.getAcc(-0.5).isZero());
  EXPECT_TRUE(h.getAcc(1.1).isZero());
}

TEST(HermiteUnitTest, CopyTest) {
  HermiteUnit<1> h{{1}, {-0.5}, {0}, {4}};
  HermiteUnit<1> h2{h};
  EXPECT_TRUE(h2.getAcc(-0.5).isZero());
  EXPECT_TRUE(h2.getAcc(1.1).isZero());
}

TEST(HermiteUnitTest, AssignTest) {
  HermiteUnit<1> h{{1}, {-0.5}, {0}, {4}};
  HermiteUnit<1> h2{{0.3}, {-2.2}, {1}, {-56}};

  h2 = h;

  EXPECT_TRUE(h2.getAcc(-0.5).isZero());
  EXPECT_TRUE(h2.getAcc(1.1).isZero());
}
