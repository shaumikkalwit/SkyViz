cmake_minimum_required(VERSION 3.8)
project(drone_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/drone_viz/main_panel.hpp
  include/drone_viz/waypoint_tool.hpp
)

add_library(${PROJECT_NAME} SHARED
  src/main_panel.cpp 
  src/clicked_point_marker.cpp
  src/waypoint_tool.cpp
  ${MOC_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rviz_common
  std_srvs
  rclcpp
  rviz_common
  rviz_rendering
  geometry_msgs
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
# install(FILES icons/classes/DemoPanel.png
#         DESTINATION share/${PROJECT_NAME}/icons/classes
# )
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

add_executable(clicked_point_marker
  src/clicked_point_marker.cpp
)

ament_target_dependencies(clicked_point_marker
  rclcpp
  geometry_msgs
  visualization_msgs
  std_srvs
)

target_include_directories(clicked_point_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS clicked_point_marker
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()