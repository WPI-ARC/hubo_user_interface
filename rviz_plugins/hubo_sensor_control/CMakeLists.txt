cmake_minimum_required(VERSION 2.8.3)
project(hubo_sensor_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED teleop_msgs message_generation)


## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy teleop_msgs message_runtime
    DEPENDS systemlib QtCore QtGui )

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/hubo_sensor_control.h
  include/hubo_defaults.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/hubo_sensor_control_plugin.cpp
  src/robot_state_tab.cpp
  src/robot_state_tab_slots.cpp
  src/robot_vision_tab.cpp
  src/robot_vision_tab_slots.cpp
  src/data_request_tab.cpp
  src/data_request_tab_slots.cpp
  src/config_tab.cpp
  src/config_tab_slots.cpp
  include/hubo_sensor_control.h
  include/hubo_defaults.h
  ${MOC_FILES}
)

include_directories( include )

## Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} ${SOURCE_FILES} )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
