# Welcome to hermite

Hermite is a C++ library for computing cubic Hermite splines and natural cubic splines (mainly for robotics).

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

### Local installation

To install locally, you can clone the repository, then copy the folders containing the source code (i.e. the `include/hermite` folder) into your codebase.

## Example usage

Below is example code:

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
  svector::Vector<1> vel1 = h.getVel(-1.5);
  svector::Vector<1> acc1 = h.getAcc(-1.5);

  std::cout << pos1.toString() << std::endl; // <-0.375>
  std::cout << vel1.toString() << std::endl; // <1.75>
  std::cout << acc1.toString() << std::endl; // <0.333>
}
```
