cmake_minimum_required(VERSION 3.8)
project(ladar_ai)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Workaround: rosidl generators don't include dependency headers on this system
# Add std_msgs include to all generated targets
set(DEP_INCLUDE_DIRS
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ZoneDistances.msg"
  "msg/Detection.msg"
  "msg/Detections.msg"
  "msg/TTSRequest.msg"
  "msg/SystemStatus.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Force include dirs onto rosidl generated targets
foreach(suffix __rosidl_generator_c __rosidl_generator_cpp
         __rosidl_typesupport_c __rosidl_typesupport_cpp
         __rosidl_typesupport_fastrtps_c __rosidl_typesupport_fastrtps_cpp
         __rosidl_typesupport_introspection_c __rosidl_typesupport_introspection_cpp)
  if(TARGET ${PROJECT_NAME}${suffix})
    get_target_property(_type ${PROJECT_NAME}${suffix} TYPE)
    if(_type STREQUAL "INTERFACE_LIBRARY")
      target_include_directories(${PROJECT_NAME}${suffix} INTERFACE ${DEP_INCLUDE_DIRS})
    else()
      target_include_directories(${PROJECT_NAME}${suffix} PUBLIC ${DEP_INCLUDE_DIRS})
    endif()
  endif()
endforeach()

install(DIRECTORY config frontend models launch scripts worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules (yolo_engine, utils, etc.)
install(DIRECTORY src/ladar_ai/
  DESTINATION local/lib/python3.10/dist-packages/ladar_ai
  FILES_MATCHING PATTERN "*.py"
  PATTERN "msg" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install Python nodes as executables
install(PROGRAMS
  scripts/run_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_zone_node
)

set(_NODES camera_detect_node voice_node fusion_decision_node tts_node web_node video_player_node laser_merger_node obstacle_avoidance_node)
foreach(_node ${_NODES})
    file(COPY scripts/run_node.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    file(RENAME ${CMAKE_CURRENT_BINARY_DIR}/run_node.py ${CMAKE_CURRENT_BINARY_DIR}/${_node})
    install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${_node} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
