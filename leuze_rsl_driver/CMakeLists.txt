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

include_directories(include
  ${Boost_INCLUDES}
)


#------------------SIMULATION---------------------------
set(SIMULATION "no")     #Set to "yes" for simulation mode



#------------------RSL400---------------------------

add_library( rsl400_def_lib INTERFACE )

if(SIMULATION STREQUAL "yes")
  target_compile_definitions( rsl400_def_lib INTERFACE -DRSL400 -DSIMULATION)
  add_executable(leuze_rsl400_driver src/leuze_rsl_driver.cpp
                    src/rsl400/rsl400_interface.cpp
                    src/udp_sim.cpp
                    )
else()
  target_compile_definitions( rsl400_def_lib INTERFACE -DRSL400 )
  add_executable(leuze_rsl400_driver src/leuze_rsl_driver.cpp
                    src/rsl400/rsl400_interface.cpp
                    )
endif()

ament_target_dependencies(leuze_rsl400_driver 
                    rclcpp 
                    sensor_msgs
                    leuze_msgs
                    angles
                  )
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

#ToDo: Add unit tests for ROS2 (This part still uses CATKIN from ROS1!)
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(test_parse_data test/test_parse_data.test test/test_parse_data.cpp src/rsl400/rsl400_interface.cpp)
    target_link_libraries(test_parse_data ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    add_dependencies(test_parse_data ${catkin_EXPORTED_TARGETS})
endif()
