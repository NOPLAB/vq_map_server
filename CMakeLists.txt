cmake_minimum_required(VERSION 3.16)
project(vq_map_server LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VqMap.msg"
  "srv/GetVqMap.srv"
  DEPENDENCIES std_msgs)
rosidl_get_typesupport_target(vq_map_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(vq_map_core src/vq_map.cpp)
target_include_directories(vq_map_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_library(vq_map_io src/map_io.cpp)
target_link_libraries(vq_map_io PUBLIC vq_map_core PRIVATE ${HDF5_C_LIBRARIES})
target_include_directories(vq_map_io SYSTEM PRIVATE ${HDF5_INCLUDE_DIRS})
add_library(vq_map_conversion src/map_converter.cpp)
target_link_libraries(vq_map_conversion PUBLIC vq_map_core PRIVATE ${PCL_LIBRARIES})
target_include_directories(vq_map_conversion SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

add_executable(vq_map_convert src/convert_map.cpp)
target_link_libraries(vq_map_convert vq_map_conversion vq_map_io yaml-cpp)
add_executable(vq_map_inspect src/inspect_map.cpp)
target_link_libraries(vq_map_inspect vq_map_io)
add_executable(vq_map_server_node src/vq_map_server_node.cpp)
target_link_libraries(vq_map_server_node vq_map_conversion vq_map_io "${vq_map_typesupport}")
ament_target_dependencies(vq_map_server_node rclcpp rcl_interfaces std_msgs sensor_msgs)

foreach(target vq_map_core vq_map_io vq_map_conversion
               vq_map_convert vq_map_inspect vq_map_server_node)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vq_map test/test_vq_map.cpp)
  target_link_libraries(test_vq_map vq_map_core)
  ament_add_gtest(test_map_io test/test_map_io.cpp)
  target_link_libraries(test_map_io vq_map_io ${HDF5_C_LIBRARIES})
  target_include_directories(test_map_io SYSTEM PRIVATE ${HDF5_INCLUDE_DIRS})
  ament_add_gtest(test_map_converter test/test_map_converter.cpp)
  target_link_libraries(test_map_converter vq_map_conversion ${PCL_LIBRARIES})
  target_include_directories(test_map_converter SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
  ament_add_gtest(test_cli test/test_cli.cpp TIMEOUT 60)
  target_link_libraries(test_cli vq_map_io)
  target_compile_definitions(test_cli PRIVATE
    TEST_CONVERT_PATH="$<TARGET_FILE:vq_map_convert>"
    TEST_INSPECT_PATH="$<TARGET_FILE:vq_map_inspect>")
  add_dependencies(test_cli vq_map_convert vq_map_inspect)
  ament_add_gtest(test_ros_node test/test_ros_node.cpp TIMEOUT 90
    APPEND_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}")
  target_link_libraries(test_ros_node vq_map_io "${vq_map_typesupport}")
  ament_target_dependencies(test_ros_node rclcpp std_msgs sensor_msgs)
  target_compile_definitions(test_ros_node PRIVATE
    TEST_SERVER_PATH="$<TARGET_FILE:vq_map_server_node>")
  add_dependencies(test_ros_node vq_map_server_node)
  foreach(target test_vq_map test_map_io test_map_converter test_cli test_ros_node)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endforeach()
endif()

install(TARGETS vq_map_core vq_map_io vq_map_conversion
  EXPORT export_vq_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS vq_map_convert vq_map_inspect vq_map_server_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE NOTICE DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_vq_map HAS_LIBRARY_TARGET)
ament_export_dependencies(HDF5 PCL rosidl_default_runtime)
ament_package()
