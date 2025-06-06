cmake_minimum_required(VERSION 3.5)
project(leuze_msgs)

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
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  # Include Lint
  find_package(ament_lint_auto REQUIRED)

  # The following line prevents the linter from requiring the presence of copyrights and license in each source file
  set(ament_cmake_copyright_FOUND TRUE)

  # Uncomment the following line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExtendedStatusProfileMsgRsl200.msg"
  "msg/ExtendedStatusProfileMsgRsl400.msg"
  "msg/PhidgetIKInputMsg.msg"
  "msg/PhidgetIKOutputMsg.msg"
  DEPENDENCIES std_msgs
)

ament_package()

