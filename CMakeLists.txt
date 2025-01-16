cmake_minimum_required(VERSION 3.16.0)
project(rb5_ros_wrapper)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
  message_generation
)

find_package(Boost REQUIRED)

add_message_files(
  FILES rx.msg
)

add_action_files(
  DIRECTORY action
  FILES Motion.action
)

add_service_files(
  FILES
  SearchAruco.srv
)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(rb5_wrapper src/wrapper.cpp src/lanros2podo.cpp)
target_link_libraries(rb5_wrapper ${catkin_LIBRARIES})

add_executable(rb5_update src/update.cpp src/lanpodo2ros.cpp)
target_link_libraries(rb5_update ${catkin_LIBRARIES})

add_executable(rb5_client src/RB5_Client.cpp)
target_link_libraries(rb5_client ${catkin_LIBRARIES})

add_executable(cobot_controller src/main.cpp)
target_link_libraries(cobot_controller ${catkin_LIBRARIES})

add_dependencies(rb5_wrapper ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(rb5_update ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(rb5_client ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(cobot_controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


catkin_install_python(PROGRAMS
  src/pythons/commandPosition.py
  src/pythons/keyInput.py
  src/pythons/searchServer.py
  src/pythons/tossBridge_not_in_use.py
  src/pythons/tossPlanner_not_in_use.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
    geometry_msgs
    message_runtime
)
