/**
 * @file
 *
 * First example provided in the README
 */

#include <cstddef>
#include <iostream>

#include <hermite/hermite.hpp>

int main() {
  // create hermite object
  hermite::Hermite<1> h;

  // create poses
  hermite::Pose<1> p0{-3, {-2}, {0}};
  hermite::Pose<1> p1{0, {2}, {1}};
  hermite::Pose<1> p2{2, {3}, {2}};
  hermite::Pose<1> p3{6, {0}, {0}};

  // add poses into hermite object
  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  // calculate position, velocity, acceleration vectors
  svector::Vector<1> pos1 = h.getPos(-1.5);
  svector::Vector<1> pos2 = h.getPos(3.5);
  svector::Vector<1> vel1 = h.getVel(-1.5);
  svector::Vector<1> vel2 = h.getVel(3.5);
  svector::Vector<1> acc1 = h.getAcc(-1.5);
  svector::Vector<1> acc2 = h.getAcc(3.5);

  std::cout << pos1.toString() << std::endl; // <-0.375>
  std::cout << pos2.toString() << std::endl; // <3.223>
  std::cout << vel1.toString() << std::endl; // <1.75>
  std::cout << vel2.toString() << std::endl; // <-1.211>
  std::cout << acc1.toString() << std::endl; // <0.333>
  std::cout << acc2.toString() << std::endl; // <-1.156>

  // looping through positions
  double curTime = h.getLowestTime();
  double timestep = 0.01;
  while (curTime <= h.getHighestTime()) {
    std::cout << h.getPos(curTime).toString() << std::endl;
    curTime += timestep;
  }
}
