@mainpage

Hermite is a C++ library for computing cubic Hermite splines and natural cubic splines (mainly for robotics).

# Splines background

Splines are parametric piecewise functions that describe an object's position given a time. A cubic spline describes this motion using a cubic polynomial function for each dimension of the position. To compute splines, it needs to be given a path. Path are given through a hermite::Pose object. A pose (for our purposes) is a data structure containing a point in time, where each pose describes the time of the pose, the position at that time, and the velocity at that time.

Cubic hermite splines allow for more local control, giving users the ability so specify both desired positions and velocities at a certain time. Changing one waypoint will only affect the curve that goes through the waypoint and will not affect other parts of the curve. However, each waypoint only offers C1 continuity, which means that only the function and the derivative are continuous at that point. This can lead to high jerk as acceleration is discontinuous, and the curve is not as smooth. On the other hand, natural cubic splines offer C2 continuity at each waypoint, meaning that the function, its derivative, and its second derivative are continuous. This not only makes the curve more smooth but also reduces the jerk as the acceleration is continuous. However, the user loses local control, as editing one waypoint will regenerate the entire curve rather than the curve around the waypoint, and the user cannot specify the velocities of the poses besides the first and the last one.

## Hermite splines

Each "piece" of a cubic hermite spline can be given by the following equation:

@f[
\mathbf{p}(t) = h_{00}(t)\mathbf{p_0} + h_{10}(t)\left(x_{k+1} - x_k\right)\mathbf{m_0} + h_{01}(t)\mathbf{p_1} + h_{11}(t)\left(x_{k+1} - x_k\right)\mathbf{m_1}
@f]

Where:

@f[
h_{00}(t)=2t^3-3t^2+1 \\
h_{10}(t)=t^3-2t^2+t \\
h_{01}(t)=-2t^3+3t^2 \\
h_{11}(t)=t^3-t^2
@f]

From this, we can see that we can generate a smooth cubic spline on any arbitrary interval given the interval's starting and ending times, positions, and velocities. Since @f$\mathbf{p}@f$ is a vector, this can be applied to any dimension, generating a smooth path not only for 1D objects but 2D and 3D as well. To generate a spline for many waypoints, we need to specify the time, position, and velocity for every waypoint.

**Important things to remember for Hermite splines**:

- They allow for local control
- They only offer C1 continuity
- You need to specify time, position, and velocity for every waypoint

## Natural cubic splines

Natural cubic splines are Hermite splines, but the velocities at each waypoint are automatically determined to ensure continuous acceleration. In one dimension, each "piece" of a natural cubic spline can be described by:

@f[
p(t)=a+bt+ct^2+dt^3 \\
p'(t)=b+2ct+3dt^2 \\
p''(t)=2c+6dt
@f]

At each waypoint, we need to make sure that @f$p(t)@f$, @f$p'(t)@f$, and @f$p''(t)@f$, forming a system of equations. In order to fully solve this system of equations, the user needs to provide the time and position of each waypoint and the velocities of the first and last waypoints. This can be extended to multiple dimensions, allowing for the generation of a smooth path in 1D, 2D, and 3D.

**Important things to remember for natural cubic splines**:

- They do not allow for local control
- They offer C2 continuity
- You need to specify time and position for every waypoint and the velocity for the first and last waypoints

# Hermite library

Below gives information about usage of the library.

## Installation

Install the library:

```sh
$ cmake -B build -DCMAKE_BUILD_TYPE=Release
$ cmake --build build --target install
```

Then place the code below:

```cmake
cmake_minimum_required(VERSION 3.16.3)
project(MyProject LANGUAGES CXX)
 
find_package(hermite REQUIRED)
 
add_executable(main main.cpp)
target_link_libraries(main PRIVATE hermite::hermite)
```

The include path will be like so:
```cpp
#include <hermite/cubic.hpp>   // for cubic splines
#include <hermite/hermite.hpp> // for hermite splines
```

## Basic Library Usage

The main class that is used is the hermite::Hermite class. This class acts like a std::map, where different hermite::Pose waypoint objects can be inserted and removed with logarithmic complexity. Using this class, you need to specify the multiplier for the time. The higher this multiplier, the more accurate the time is stored. However, in the case of robotics, a multiplier of 10 works as time to the first decimal digit is precise enough. Because of this, the default constructor initializes the class with a multiplier of 10. For more information about the multiplier, visit the documentation about the class, which can be accessed above.

After inserting the waypoints, you can get the position, velocity, and acceleration vectors of the path at any given time within the time interval (from the time given in the first waypoint to the time given in the last waypoint). You would do this by calling hermite::Hermite::getPos() (or hermite::Hermite::operator()()), hermite::Hermite::getVel(), and hermite::Hermite::getAcc(), respectively. These methods return an svector::Vector object.

The hermite::Hermite and hermite::Pose classes comes with a template, where you have to specify the number of dimensions of each points. For example, if you were working in a 2D space, then you would put "2" in the template. The examples below are in 1 dimension, hence the "1" in the template argument.

Example usage:

```cpp
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
```

Additional utility methods can be found by visiting the class's documentation, but the ones listed above are most likely to be used on a robot.

### Cubics

hermite::Cubic is the main class for the use of cubic splines. It is highly recommended to manage waypoints (inserting, deleting, replacing) through the hermite::Hermite class, then using hermite::Hermite::getAllWaypoints() to feed into the constructor of hermite::Cubic. This is because the constructor of hermite::Cubic expects a vector of hermite::Pose waypoints that are sorted by time and that do not contain any repeats of times. However, to conserve memory, you can directly pass in a vector to the constructor, but you need to make sure that **the waypoints in the vector are sorted by time, and there are no two waypoints that share the same time.**

Unlike hermite::Hermite objects, each hermite::Cubic object can only generate a path for a unique set of waypoints. In other words, it is impossible to generate multiple paths from multiple combinations of waypoints using a single hermite::Cubic object, as you cannot insert or delete waypoints from the object. To generate a new spline for a new set of waypoints, you would need to insert points into or delete points from a hermite::Hermite object then create a new hermite::Cubic object by calling the constructor with the output of hermite::Hermite::getAllWaypoints().

The hermite::Cubic and hermite::Pose classes comes with a template, where you have to specify the number of dimensions of each points. For example, if you were working in a 2D space, then you would put "2" in the template. The examples below are in 1 dimension, hence the "1" in the template argument.

Example usage:

```cpp
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
```

Additional utility methods can be found by visiting the class's documentation, but the ones listed above are most likely to be used on a robot.
