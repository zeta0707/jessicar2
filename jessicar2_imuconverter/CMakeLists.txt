cmake_minimum_required(VERSION 2.8.3)
project(jessicar2_imuconverter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jessicar2_imuconverter
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(jessicar2_imuconverter_node src/mpu6050_imu_converter_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(jessicar2_imuconverter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(jessicar2_imuconverter_node
   ${catkin_LIBRARIES}
)


