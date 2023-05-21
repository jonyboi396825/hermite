#include <gtest/gtest.h>

#include "hermite/hermite_sub.hpp"

using namespace hermite;

// The out of bounds test still remain because I used to return a zero vector if
// out of bounds, but I removed that, so the tests ensure that I actually
// removed that check.

TEST(HermiteSubTest, PosTestUnit) {
  HermiteSub<1> h{{0}, {2.5}, {-3.8}, {0}, 0, 1};
  EXPECT_NEAR(h.getPos(0)[0], 0, 0.00001);
  EXPECT_NEAR(h.getPos(0.1)[0], -0.2378, 0.00001);
  EXPECT_NEAR(h.getPos(0.5)[0], 0.775, 0.00001);
  EXPECT_NEAR(h.getPos(0.75)[0], 1.93125, 0.00001);
  EXPECT_NEAR(h.getPos(1)[0], 2.5, 0.00001);
}

TEST(HermiteSubTest, PosOutOfBoundTestUnit) {
  HermiteSub<1> h{{0}, {2.6}, {-3.8}, {0}, 0, 1};
  EXPECT_FALSE(h.getPos(-50).isZero());
  EXPECT_FALSE(h.getPos(50).isZero());
}

TEST(HermiteSubTest, VelTestUnit) {
  HermiteSub<1> h{{3}, {1.5}, {2.8}, {1}, 0, 1};
  EXPECT_NEAR(h.getVel(0)[0], 2.8, 0.00001);
  EXPECT_NEAR(h.getVel(0.1)[0], 0.784, 0.00001);
  EXPECT_NEAR(h.getVel(0.5)[0], -3.2, 0.00001);
  EXPECT_NEAR(h.getVel(0.75)[0], -2.375, 0.00001);
  EXPECT_NEAR(h.getVel(1)[0], 1, 0.00001);
}

TEST(HermiteSubTest, VelOutOfBoundTestUnit) {
  HermiteSub<1> h{{3}, {1.5}, {2.8}, {1}, 0, 1};
  EXPECT_FALSE(h.getVel(-1).isZero());
  EXPECT_FALSE(h.getVel(2).isZero());
}

TEST(HermiteSubTest, AccTestUnit) {
  HermiteSub<1> h{{1}, {-0.5}, {0}, {4}, 0, 1};
  EXPECT_NEAR(h.getAcc(0)[0], -17, 0.00001);
  EXPECT_NEAR(h.getAcc(0.1)[0], -12.8, 0.00001);
  EXPECT_NEAR(h.getAcc(0.5)[0], 4, 0.00001);
  EXPECT_NEAR(h.getAcc(0.75)[0], 14.5, 0.00001);
  EXPECT_NEAR(h.getAcc(1)[0], 25, 0.00001);
}

TEST(HermiteSubTest, AccOutOfBoundTestUnit) {
  HermiteSub<1> h{{1}, {-0.5}, {0}, {4}, 0, 1};
  EXPECT_FALSE(h.getAcc(-0.5).isZero());
  EXPECT_FALSE(h.getAcc(1.1).isZero());
}

TEST(HermiteSubTest, PosTest) {
  HermiteSub<1> h{{0}, {2.5}, {-3.8}, {0}, 4, 7};
  EXPECT_NEAR(h.getPos(4)[0], 0, 0.01);
  EXPECT_NEAR(h.getPos(4.7)[0], -1.218, 0.01);
  EXPECT_NEAR(h.getPos(5.45)[0], -0.284, 0.01);
  EXPECT_NEAR(h.getPos(6.88)[0], 2.471, 0.01);
  EXPECT_NEAR(h.getPos(7)[0], 2.5, 0.01);
}

TEST(HermiteSubTest, PosOutOfBoundTest) {
  HermiteSub<1> h{{0}, {2.6}, {-3.8}, {0}, 4, 7};
  EXPECT_FALSE(h.getPos(3.5).isZero());
  EXPECT_FALSE(h.getPos(10).isZero());
}

TEST(HermiteSubTest, VelTest) {
  HermiteSub<1> h{{3}, {1.5}, {2.8}, {1}, 3, 5};
  EXPECT_NEAR(h.getVel(3)[0], 2.8, 0.01);
  EXPECT_NEAR(h.getVel(3.1)[0], 1.954, 0.01);
  EXPECT_NEAR(h.getVel(3.7)[0], -1.447, 0.01);
  EXPECT_NEAR(h.getVel(4.4)[0], -1.799, 0.01);
  EXPECT_NEAR(h.getVel(5)[0], 1, 0.01);
}

TEST(HermiteSubTest, VelOutOfBoundTest) {
  HermiteSub<1> h{{3}, {1.5}, {2.8}, {1}, 3, 5};
  EXPECT_FALSE(h.getVel(-1).isZero());
  EXPECT_FALSE(h.getVel(5.1).isZero());
}

TEST(HermiteSubTest, AccTest) {
  HermiteSub<1> h{{1}, {-0.5}, {0}, {4}, -3, 1};
  EXPECT_NEAR(h.getAcc(-3)[0], -2.5625, 0.01);
  EXPECT_NEAR(h.getAcc(-2.4)[0], -1.49375, 0.01);
  EXPECT_NEAR(h.getAcc(-1.1)[0], 0.821875, 0.01);
  EXPECT_NEAR(h.getAcc(0)[0], 2.78125, 0.01);
  EXPECT_NEAR(h.getAcc(1)[0], 4.5625, 0.01);
}

TEST(HermiteSubTest, AccOutOfBoundTest) {
  HermiteSub<1> h{{1}, {-0.5}, {0}, {4}, -3, 1};
  EXPECT_FALSE(h.getAcc(-3.5).isZero());
  EXPECT_FALSE(h.getAcc(1.1).isZero());
}

TEST(HermiteSubTest, CopyTest) {
  HermiteSub<1> htmp{{1}, {-0.5}, {0}, {4}, -3, 1};
  HermiteSub<1> h{htmp};

  EXPECT_NEAR(h.getAcc(-3)[0], -2.5625, 0.01);
  EXPECT_NEAR(h.getAcc(-2.4)[0], -1.49375, 0.01);
  EXPECT_NEAR(h.getAcc(-1.1)[0], 0.821875, 0.01);
  EXPECT_NEAR(h.getAcc(0)[0], 2.78125, 0.01);
  EXPECT_NEAR(h.getAcc(1)[0], 4.5625, 0.01);
}

TEST(HermiteSubTest, AssignTest) {
  HermiteSub<1> htmp{{1}, {-0.5}, {0}, {4}, -3, 1};
  HermiteSub<1> h{{0}, {2}, {1}, {0.44}, -1.1, 4};

  h = htmp;

  EXPECT_NEAR(h.getAcc(-3)[0], -2.5625, 0.01);
  EXPECT_NEAR(h.getAcc(-2.4)[0], -1.49375, 0.01);
  EXPECT_NEAR(h.getAcc(-1.1)[0], 0.821875, 0.01);
  EXPECT_NEAR(h.getAcc(0)[0], 2.78125, 0.01);
  EXPECT_NEAR(h.getAcc(1)[0], 4.5625, 0.01);
}
