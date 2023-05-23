#include <iostream>

#include <hermite/hermite.hpp>

namespace spl = hermite;

int main() {
  // create hermite object
  spl::Hermite<1> h;

  // create poses
  spl::Pose<1> p0{-3, {-2}, {0}};
  spl::Pose<1> p1{0, {2}, {1}};
  spl::Pose<1> p2{2, {3}, {2}};
  spl::Pose<1> p3{6, {0}, {0}};

  // add poses into hermite object
  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  // calculate position
  auto pos1 = h.getPos(-1.5); // <-0.375>
  auto pos2 = h.getPos(3.5);  // <3.223>
  auto vel1 = h.getVel(-1.5); // <-0.375>
  auto vel2 = h.getVel(3.5);  // <3.223>
  auto acc1 = h.getAcc(-1.5); // <-0.375>
  auto acc2 = h.getAcc(3.5);  // <3.223>

  std::cout << pos1.toString() << std::endl;
  std::cout << pos2.toString() << std::endl;
  std::cout << vel1.toString() << std::endl;
  std::cout << vel2.toString() << std::endl;
  std::cout << acc1.toString() << std::endl;
  std::cout << acc2.toString() << std::endl;
}
