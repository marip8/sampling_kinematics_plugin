cmake_minimum_required(VERSION 2.8.3)
project(sampling_kinematics_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  pluginlib
  tf2_eigen
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    pluginlib
    tf2_eigen
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/external_axis_sampling_kinematics_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  FILES
    plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_test
    test/abb_irb2400_rail/test.rostest
    test/utest.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test
    GTest::GTest
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
  )
endif()
