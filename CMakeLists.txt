cmake_minimum_required(VERSION 3.0.2)
project(rosmath)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

## TODO: make this optional
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

catkin_package(
    INCLUDE_DIRS 
        include
    LIBRARIES 
        rosmath
    CATKIN_DEPENDS 
        geometry_msgs 
        nav_msgs 
        roscpp 
        rospy 
        sensor_msgs
    DEPENDS 
        EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/conversions.cpp
  src/${PROJECT_NAME}/eigen/conversions.cpp
  src/${PROJECT_NAME}/math.cpp
  src/${PROJECT_NAME}/misc.cpp
  src/${PROJECT_NAME}/random.cpp
  src/${PROJECT_NAME}/stats.cpp
  # make this optional
  src/${PROJECT_NAME}/opencv/conversions.cpp
  src/${PROJECT_NAME}/nav_msgs/math.cpp
  src/${PROJECT_NAME}/sensor_msgs/math.cpp
  src/${PROJECT_NAME}/sensor_msgs/misc.cpp
  src/${PROJECT_NAME}/sensor_msgs/conversions.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${OpenCV_LIBS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_conversion_test_node 
    src/conversion_test.cpp
)

add_dependencies(${PROJECT_NAME}_conversion_test_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_conversion_test_node 
  ${catkin_LIBRARIES}
  rosmath
)

add_executable(${PROJECT_NAME}_math_test_node 
    src/math_test.cpp
)

add_dependencies(${PROJECT_NAME}_math_test_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_math_test_node 
  ${catkin_LIBRARIES}
  rosmath
)

add_executable(${PROJECT_NAME}_misc_test_node 
    src/misc_test.cpp
)

add_dependencies(${PROJECT_NAME}_misc_test_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_misc_test_node 
  ${catkin_LIBRARIES}
  rosmath
)

add_executable(${PROJECT_NAME}_example_minimal
    examples/minimal.cpp
)

add_dependencies(${PROJECT_NAME}_example_minimal
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_example_minimal
  ${catkin_LIBRARIES}
  rosmath
)


add_executable(${PROJECT_NAME}_example_conversions
    examples/conversions.cpp
)

add_dependencies(${PROJECT_NAME}_example_conversions
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_example_conversions
  ${catkin_LIBRARIES}
  rosmath
)

add_executable(${PROJECT_NAME}_example_namespace
    examples/namespace.cpp
)

add_dependencies(${PROJECT_NAME}_example_namespace
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_example_namespace
  ${catkin_LIBRARIES}
  rosmath
)

add_executable(${PROJECT_NAME}_example_random
    examples/random.cpp
)

add_dependencies(${PROJECT_NAME}_example_random
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_example_random
  ${catkin_LIBRARIES}
  rosmath
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rosmath.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
