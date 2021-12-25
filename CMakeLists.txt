cmake_minimum_required(VERSION 3.0.2)
project(frei0r_image)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  ddynamic_reconfigure
  dynamic_reconfigure
  message_generation
  message_runtime
  nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
)

set(
  ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references"
)

roslint_cpp()

add_service_files(
  FILES
  LoadPlugin.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  ddynamic_reconfigure
  dynamic_reconfigure
  message_runtime
  nodelet
  sensor_msgs
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/frei0r_image.cpp
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  stdc++fs
)
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

add_executable(list_frei0rs src/list_frei0rs.cpp)
target_link_libraries(list_frei0rs
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  stdc++fs
)

add_executable(select_plugin src/select_plugin.cpp)
target_link_libraries(select_plugin
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  stdc++fs
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  list_frei0rs
  select_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  nodelet_${PROJECT_NAME}.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

# the frei0r plugins
add_library(ros_image_pub MODULE src/ros_image_pub.cpp)
target_link_libraries(ros_image_pub ${catkin_LIBRARIES})
set_target_properties(ros_image_pub PROPERTIES PREFIX "")
install(TARGETS ros_image_pub LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(ros_image_sub MODULE src/ros_image_sub.cpp)
target_link_libraries(ros_image_sub ${catkin_LIBRARIES})
set_target_properties(ros_image_sub PROPERTIES PREFIX "")
# A frei0r plugin loader will need this directory, or manually copy this to an already
# listed frei0r plugin dir
install(TARGETS ros_image_sub LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
