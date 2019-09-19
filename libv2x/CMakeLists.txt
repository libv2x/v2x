cmake_minimum_required(VERSION 3.5)

project(libv2x)

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

include_directories(inc thirdparty/codec)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(libv2x_msgs REQUIRED)

file(GLOB CODEC_OBJS "thirdparty/codec/*.o")
list(REMOVE_ITEM CODEC_OBJS
  "thirdparty/codec/converter-example.o"
  "thirdparty/codec/pdu_collection.o")

add_library(v2x STATIC
  src/ieee1609dot3.cpp
  src/ieee1609dot2.cpp
  src/saej2735.cpp
  ${CODEC_OBJS})
ament_target_dependencies(v2x libv2x_msgs codec)

install(TARGETS v2x DESTINATION lib)

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
