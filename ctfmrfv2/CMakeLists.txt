cmake_minimum_required(VERSION 3.8)
project(ctfmrfv2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(CTFMRFv2 src/CTFMRFv2.cpp)
ament_target_dependencies(CTFMRFv2 rclcpp sensor_msgs)
target_link_libraries(CTFMRFv2 ${OpenCV_LIBS})
target_include_directories(CTFMRFv2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(CTFMRFv2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS CTFMRFv2
  DESTINATION lib/${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

set(CMAKE_BUILD_TYPE "Release")#以release的方式编译程序