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
find_package(nav_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VqMap2D.msg"
  "msg/VqMap3D.msg"
  "srv/GetVqMap2D.srv"
  "srv/GetVqMap3D.srv"
  DEPENDENCIES std_msgs)
rosidl_get_typesupport_target(vq_map_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

foreach(dimension 2d 3d)
  add_library(vq_map_core_${dimension} src/map${dimension}/vq_map.cpp)
  target_include_directories(vq_map_core_${dimension} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  add_library(vq_map_io_${dimension} src/map${dimension}/map_io.cpp)
  target_link_libraries(vq_map_io_${dimension}
    PUBLIC vq_map_core_${dimension} PRIVATE ${HDF5_C_LIBRARIES})
  target_include_directories(vq_map_io_${dimension} SYSTEM PRIVATE ${HDF5_INCLUDE_DIRS})
  add_library(vq_map_conversion_${dimension} src/map${dimension}/map_converter.cpp)
  target_link_libraries(vq_map_conversion_${dimension} PUBLIC vq_map_core_${dimension})
  add_executable(vq_map_convert_${dimension} src/map${dimension}/convert_map.cpp)
  target_link_libraries(vq_map_convert_${dimension}
    vq_map_conversion_${dimension} vq_map_io_${dimension} yaml-cpp)
  add_executable(vq_map_inspect_${dimension} src/map${dimension}/inspect_map.cpp)
  target_link_libraries(vq_map_inspect_${dimension} vq_map_io_${dimension})
  add_executable(vq_map_server_${dimension} src/map${dimension}/server_node.cpp)
  target_link_libraries(vq_map_server_${dimension}
    vq_map_conversion_${dimension} vq_map_io_${dimension} "${vq_map_typesupport}")
  ament_target_dependencies(vq_map_server_${dimension}
    rclcpp rcl_interfaces std_msgs sensor_msgs nav_msgs)
  foreach(target vq_map_core vq_map_io vq_map_conversion
                 vq_map_convert vq_map_inspect vq_map_server)
    target_compile_options(${target}_${dimension} PRIVATE -Wall -Wextra -Wpedantic)
  endforeach()
  install(TARGETS vq_map_core_${dimension} vq_map_io_${dimension} vq_map_conversion_${dimension}
    EXPORT export_vq_map ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
  install(TARGETS vq_map_convert_${dimension} vq_map_inspect_${dimension} vq_map_server_${dimension}
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endforeach()

target_link_libraries(vq_map_conversion_3d PRIVATE ${PCL_LIBRARIES})
target_include_directories(vq_map_conversion_3d SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(vq_map_conversion_2d PUBLIC nav2_map_server nav_msgs)
target_link_libraries(vq_map_conversion_2d PRIVATE yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(dimension 2d 3d)
    ament_add_gtest(test_vq_map_${dimension} test/map${dimension}/test_vq_map.cpp)
    target_link_libraries(test_vq_map_${dimension} vq_map_core_${dimension})
    ament_add_gtest(test_map_io_${dimension} test/map${dimension}/test_map_io.cpp)
    target_link_libraries(test_map_io_${dimension} vq_map_io_${dimension} ${HDF5_C_LIBRARIES})
    target_include_directories(test_map_io_${dimension} SYSTEM PRIVATE ${HDF5_INCLUDE_DIRS})
    ament_add_gtest(test_map_converter_${dimension} test/map${dimension}/test_map_converter.cpp)
    target_link_libraries(test_map_converter_${dimension} vq_map_conversion_${dimension})
    ament_add_gtest(test_cli_${dimension} test/map${dimension}/test_cli.cpp TIMEOUT 60)
    target_link_libraries(test_cli_${dimension} vq_map_io_${dimension})
    target_compile_definitions(test_cli_${dimension} PRIVATE
      TEST_CONVERT_PATH="$<TARGET_FILE:vq_map_convert_${dimension}>"
      TEST_INSPECT_PATH="$<TARGET_FILE:vq_map_inspect_${dimension}>")
    add_dependencies(test_cli_${dimension} vq_map_convert_${dimension} vq_map_inspect_${dimension})
    ament_add_gtest(test_ros_node_${dimension} test/map${dimension}/test_ros_node.cpp TIMEOUT 90
      APPEND_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}")
    target_link_libraries(test_ros_node_${dimension} vq_map_io_${dimension} "${vq_map_typesupport}")
    ament_target_dependencies(test_ros_node_${dimension} rclcpp std_msgs sensor_msgs nav_msgs)
    target_compile_definitions(test_ros_node_${dimension} PRIVATE
      TEST_SERVER_PATH="$<TARGET_FILE:vq_map_server_${dimension}>")
    add_dependencies(test_ros_node_${dimension} vq_map_server_${dimension})
    foreach(target test_vq_map test_map_io test_map_converter test_cli test_ros_node)
      target_compile_options(${target}_${dimension} PRIVATE -Wall -Wextra -Wpedantic)
    endforeach()
  endforeach()
  target_link_libraries(test_map_io_2d vq_map_io_3d)
  target_sources(test_cli_2d PRIVATE test/map2d/test_inspect.cpp)
  target_link_libraries(test_map_converter_3d ${PCL_LIBRARIES})
  target_include_directories(test_map_converter_3d SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE NOTICE DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_vq_map HAS_LIBRARY_TARGET)
ament_export_dependencies(HDF5 PCL nav2_map_server nav_msgs rosidl_default_runtime)
ament_package()
