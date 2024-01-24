cmake_minimum_required(VERSION 3.5)
project(leuze_phidget_driver)

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
find_package(leuze_msgs REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(include
  ${Boost_INCLUDES}
)

add_executable(leuze_phidget_driver_node src/leuze_phidget_driver_node.cpp
                                         src/leuze_phidget_driver.cpp)
ament_target_dependencies(leuze_phidget_driver_node rclcpp)

ament_target_dependencies(leuze_phidget_driver_node
                            rclcpp
                            leuze_msgs
                          )

install(
  TARGETS
    leuze_phidget_driver_node
  DESTINATION 
    lib/${PROJECT_NAME}
)

ament_package()
