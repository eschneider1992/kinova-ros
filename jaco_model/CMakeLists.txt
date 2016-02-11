cmake_minimum_required(VERSION 2.8.3)
project(jaco_model)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  urdf
  visualization_msgs
  reflex_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
   urdf/standalone_arm.xacro
   urdf/jaco_arm.xacro
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
 )
install(DIRECTORY meshes
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   FILES_MATCHING PATTERN "*.dae"
 )
#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Declare a cpp executable
add_executable(move_arm move_arm.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(move_arm ${catkin_LIBRARIES})
add_dependencies(move_arm reflex_msgs_gencpp)