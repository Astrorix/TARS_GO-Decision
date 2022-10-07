project(roborts_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  roborts_costmap
  )

find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
  )

catkin_package()

add_executable(behavior_test_node
   ${DecisionProtoHds}
   ${DecisionProtoSrc}
   aurora/back_boot_area_behavior.h
   aurora/chase_behavior.h
   aurora/chassis_limited_behavior.h
   aurora/defend_behavior.h
   aurora/escape_behavior.h
   aurora/follow_behavior.h
   aurora/frozen_behavior.h
   aurora/gain_blood_behavior.h
   aurora/gain_bullets_behavior.h
   aurora/get_supply_behavior.h
   aurora/gimbal_limited_behavior.h
   aurora/hunt_master_behavior.h
   aurora/hunt_slave_behavior.h
   aurora/search_behavior.h
   aurora/shoot_behavior.h
   aurora/turn_defend_behavior.h
   behavior_test.cpp
   executor/chassis_executor.cpp
   )

   target_link_libraries(behavior_test_node
   PRIVATE
   roborts_costmap
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   )

 add_dependencies(behavior_test_node
   roborts_msgs_generate_messages)


   add_executable(chenfs_node
   ${DecisionProtoHds}
   ${DecisionProtoSrc}
   aurora/exam.h
   aurora/back_boot_area_behavior.h
   aurora/chase_behavior.h
   aurora/chassis_limited_behavior.h
   aurora/defend_behavior.h
   aurora/escape_behavior.h
   aurora/follow_behavior.h
   aurora/frozen_behavior.h
   aurora/gain_blood_behavior.h
   aurora/gain_bullets_behavior.h
   aurora/get_supply_behavior.h
   aurora/gimbal_limited_behavior.h
   aurora/hunt_master_behavior.h
   aurora/hunt_slave_behavior.h
   aurora/search_behavior.h
   aurora/shoot_behavior.h
   aurora/turn_defend_behavior.h
   chenfs.cpp
   executor/chassis_executor.cpp
   )


 target_link_libraries(chenfs_node
   PRIVATE
   roborts_costmap
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   )

 add_dependencies(chenfs_node
   roborts_msgs_generate_messages)

catkin_package()

add_executable(dark_night_node
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  aurora/back_boot_area_behavior.h
  aurora/chase_behavior.h
  aurora/chassis_limited_behavior.h
  aurora/defend_behavior.h
  aurora/escape_behavior.h
  aurora/follow_behavior.h
  aurora/frozen_behavior.h
  aurora/gain_blood_behavior.h
  aurora/gain_bullets_behavior.h
  aurora/get_supply_behavior.h
  aurora/gimbal_limited_behavior.h
  aurora/hunt_master_behavior.h
  aurora/hunt_slave_behavior.h
  aurora/search_behavior.h
  aurora/shoot_behavior.h
  aurora/turn_defend_behavior.h
  DarkNights.cpp
  executor/chassis_executor.cpp
  )

target_link_libraries(dark_night_node
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(dark_night_node
  roborts_msgs_generate_messages)

