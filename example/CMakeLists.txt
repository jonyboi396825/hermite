add_executable(example1 example1.cpp)
add_executable(example2 example2.cpp)
target_link_libraries(example1 PRIVATE hermite)
target_link_libraries(example2 PRIVATE hermite)

