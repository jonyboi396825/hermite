#include <cstddef>
#include <iostream>

#include <hermite/hermite.hpp>
#include <hermite/cubic.hpp>

int main() {
  // create hermite object
  hermite::Hermite<1> h;

  // create poses
  hermite::Pose<1> p0{0, {1}, {2}};
  hermite::Pose<1> p1{2, {2}, {0}};
  hermite::Pose<1> p2{5, {0}, {0}};
  hermite::Pose<1> p3{8, {0}, {1}};

  // add poses into hermite object
  h.insert(p0);
  h.insert(p1);
  h.insert(p2);
  h.insert(p3);

  // create cubic object
  hermite::Cubic<1> cub{h.getAllWaypoints()};

  // calculate position vectors from cubic
  svector::Vector<1> pos1 = cub.getPos(1);
  svector::Vector<1> pos2 = cub.getPos(4);
  svector::Vector<1> vel1 = cub.getVel(1);
  svector::Vector<1> vel2 = cub.getVel(4);
  svector::Vector<1> acc1 = cub.getAcc(1);
  svector::Vector<1> acc2 = cub.getAcc(4);

  std::cout << pos1.toString() << std::endl; // <2.105>
  std::cout << pos2.toString() << std::endl; // <0.712>
  std::cout << vel1.toString() << std::endl; // <0.355>
  std::cout << vel2.toString() << std::endl; // <-0.749>
  std::cout << acc1.toString() << std::endl; // <-1.211>
  std::cout << acc2.toString() << std::endl; // <0.015>

  // looping through positions
  double curTime = cub.getLowestTime();
  double timestep = 0.01;
  while (curTime <= cub.getHighestTime()) {
    std::cout << cub.getPos(curTime).toString() << std::endl;
    curTime += timestep; 
  }
}
