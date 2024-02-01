cmake_minimum_required(VERSION 3.5)
project(leuze_rsl_driver)

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

# find dependencies
find_package(Boost COMPONENTS thread REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(leuze_msgs REQUIRED)
find_package(angles REQUIRED)

option(BUILD_TESTING "Build tests" OFF)

if(BUILD_TESTING)
  # Include Lint
  find_package(ament_lint_auto REQUIRED)

  # Exclude all the c++ implementation files in test directory
  file(GLOB_RECURSE AMENT_LINT_AUTO_FILE_EXCLUDE
    test/*.cpp
  )

  # The following line prevents the linter from requiring the presence of copyrights and license in each source file
  set(ament_cmake_copyright_FOUND TRUE)

  # Uncomment the following line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Include gtest:
  find_package(ament_cmake_gtest REQUIRED)

  # Add test source files
  ament_add_gtest(${PROJECT_NAME}_test test/test_parse_data.cpp src/rsl400/rsl400_interface.cpp)

  # Add test include directories
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # Add test dependencies
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    sensor_msgs
    leuze_msgs
    angles
    Boost
  )
endif()

include_directories(include
  ${Boost_INCLUDES}
)


#------------------SIMULATION---------------------------
set(SIMULATION "yes")     #Set to "yes" for simulation mode



#------------------RSL400---------------------------

add_library( rsl400_def_lib INTERFACE )

if(SIMULATION STREQUAL "yes")
  target_compile_definitions(rsl400_def_lib INTERFACE -DRSL400 -DSIMULATION)
  add_executable(leuze_rsl400_driver src/leuze_rsl_driver.cpp src/rsl400/rsl400_interface.cpp src/udp_sim.cpp)
else()
  target_compile_definitions( rsl400_def_lib INTERFACE -DRSL400 )
  add_executable(leuze_rsl400_driver src/leuze_rsl_driver.cpp src/rsl400/rsl400_interface.cpp)
endif()

ament_target_dependencies(leuze_rsl400_driver rclcpp sensor_msgs leuze_msgs angles)
target_link_libraries(leuze_rsl400_driver Boost::thread rsl400_def_lib)




add_executable(
  node_inheritance
  src/node_inheritance.cpp
)
ament_target_dependencies(
  node_inheritance
  rclcpp
)


install(
  TARGETS
    leuze_rsl400_driver
    node_inheritance
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
