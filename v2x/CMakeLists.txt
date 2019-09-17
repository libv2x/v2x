cmake_minimum_required(VERSION 3.5)

project(v2x)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(../libv2x/inc ../libv2x/thirdparty/codec)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(libv2x_msgs REQUIRED)
find_package(libv2x REQUIRED)
find_library(LIBV2X libv2x.a REQUIRED)

# IEEE 1609.3 - DevEmu
add_executable(${PROJECT_NAME}_ieee1609dot3_devemu
  src/ieee1609dot3_devemu.cpp
)
ament_target_dependencies(${PROJECT_NAME}_ieee1609dot3_devemu
  rclcpp
  libv2x_msgs
)

# IEEE 1609.3 - Host
add_executable(${PROJECT_NAME}_ieee1609dot3
  src/ieee1609dot3.cpp
)
target_link_libraries(${PROJECT_NAME}_ieee1609dot3 ${LIBV2X})
ament_target_dependencies(${PROJECT_NAME}_ieee1609dot3
  rclcpp
  libv2x_msgs
  libv2x
)

# IEEE 1609.2 - Host
add_executable(${PROJECT_NAME}_ieee1609dot2
  src/ieee1609dot2.cpp
)
target_link_libraries(${PROJECT_NAME}_ieee1609dot2 ${LIBV2X})
ament_target_dependencies(${PROJECT_NAME}_ieee1609dot2
  rclcpp
  libv2x_msgs
  libv2x
)

install(TARGETS
  # IEEE 1609.3 - DevEmu
  ${PROJECT_NAME}_ieee1609dot3_devemu
  # IEEE 1609.3 - Host
  ${PROJECT_NAME}_ieee1609dot3
  # IEEE 1609.2 - Host
  ${PROJECT_NAME}_ieee1609dot2
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
