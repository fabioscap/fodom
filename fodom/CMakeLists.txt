cmake_minimum_required(VERSION 3.16)
project(fodom)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# Single shared library — add more component sources here as the project grows
add_library(fodom SHARED
  src/kdtree.cpp
  src/imu.cpp
  src/odom.cpp
)
target_include_directories(fodom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fodom PUBLIC Eigen3::Eigen Threads::Threads)

include(FetchContent)
FetchContent_Declare(
  polyscope
  GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
  GIT_TAG        v2.3.0
)
FetchContent_MakeAvailable(polyscope)

# add_subdirectory(app)



install(TARGETS fodom EXPORT fodomTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT fodomTargets
  FILE fodomTargets.cmake
  NAMESPACE fodom::
  DESTINATION lib/cmake/fodom
)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/fodomConfig.cmake"
"include(CMakeFindDependencyMacro)\nfind_dependency(Eigen3)\nfind_dependency(Threads)\ninclude(\"\${CMAKE_CURRENT_LIST_DIR}/fodomTargets.cmake\")\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/fodomConfig.cmake"
  DESTINATION lib/cmake/fodom
)
