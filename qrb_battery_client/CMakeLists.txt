cmake_minimum_required(VERSION 3.8)
project(qrb_battery_client VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(DBUS REQUIRED dbus-1)

include_directories(
  include
  ${DBUS_INCLUDE_DIRS}
)

# add targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/battery_client.cpp
)

link_directories(${DBUS_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME} ${DBUS_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(BUILD_TESTING)

  ament_auto_add_executable(battery_client_test
    test/src/battery_client_test.cpp
  )

  target_link_libraries(battery_client_test
    ${PROJECT_NAME}
  )

endif()

ament_auto_package()