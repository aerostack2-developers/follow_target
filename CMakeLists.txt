cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME follow_target)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  std_msgs
  as2_msgs
  std_srvs
  geometry_msgs
  tf2
  motion_reference_handlers
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES 
  src/follow_target.cpp
  lib/ft_base.cpp
  lib/ft_speed_controller.cpp
  lib/ft_utils.cpp
  lib/ft_pickup.cpp
  # lib/ft_dynamic_land.cpp
)

# set(INCLUDE_HPP_FILES 
#   include/${PROJECT_NAME}/follow_target.hpp
# )

add_executable(${PROJECT_NAME}_node src/follow_target_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
    
ament_package()
