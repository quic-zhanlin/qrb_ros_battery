cmake_minimum_required(VERSION 3.8)
project(qrb_ros_battery VERSION 1.0.0)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PkgConfig REQUIRED)
# libdbus-1-dev
pkg_search_module(DBUS REQUIRED dbus-1)

include_directories(
  include
  ${DBUS_INCLUDE_DIRS}
)

ament_auto_add_executable(battery_node
  src/battery_node.cpp
)

target_link_libraries(battery_node
  ${DBUS_LIBRARIES}
)

ament_auto_package()

