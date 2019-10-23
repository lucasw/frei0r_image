cmake_minimum_required(VERSION 2.8.3)
project(frei0r_image)

add_compile_options(-std=c++1z)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  # ddynamic_reconfigure
  message_generation
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

find_package(Ceres REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS ddynamic_reconfigure
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_library(frei0r_image
  src/frei0r_image.cpp
)
# add_dependencies(frei0r_image ${PROJECT_NAME}_gencfg)
target_link_libraries(frei0r_image
  ${catkin_LIBRARIES}
  stdc++fs
)
# add_dependencies(frei0r_image ${PROJECT_NAME}_gencpp)

install(TARGETS frei0r_image
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  nodelet_${PROJECT_NAME}.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

