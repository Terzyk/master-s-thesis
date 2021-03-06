cmake_minimum_required(VERSION 3.0.2)
project(pracownia_badawcza)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
  map_server
  nav_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  ompl
  message_filters
  tf
  tf_conversions
  message_generation
)

find_package(OMPL)

add_service_files(
  FILES
  AddTwoInts.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_library
  CATKIN_DEPENDS map_server nav_msgs roscpp rospy std_msgs visualization_msgs message_filters  tf tf_conversions
#  DEPENDS system_lib
)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)





## Declare a C++ library
file(GLOB pracownia_badawcza_sources
    src/ompl_lib.cpp
)

file(GLOB pracownia_badawcza_headers
    include/pracownia_badawcza/ompl_lib/*.hpp
)


add_library(${PROJECT_NAME}_library STATIC ${pracownia_badawcza_sources} ${pracownia_badawcza_headers}
)
add_dependencies(${PROJECT_NAME}_library pracownia_badawcza_gencpp)
target_link_libraries(${PROJECT_NAME}_library
   ${catkin_LIBRARIES}
)


add_executable(map_node src/map_node.cpp)
add_dependencies(map_node pracownia_badawcza_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_node 
	${PROJECT_NAME}_library
	${catkin_LIBRARIES}
	ompl
)



add_executable(points src/points.cpp)
target_link_libraries(points ${catkin_LIBRARIES})


add_executable(local_plan src/local_plan.cpp)
target_link_libraries(local_plan ${catkin_LIBRARIES})
add_dependencies(local_plan pracownia_badawcza_gencpp)






set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

catkin_install_python(PROGRAMS src/add_two_ints_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/neural_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


