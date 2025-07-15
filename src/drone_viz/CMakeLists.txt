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

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/drone_viz/main_panel.hpp
)

add_library(main_panel src/main_panel.cpp 
  waypoint_tool src/waypoint_tool.cpp
  ${MOC_FILES})
target_include_directories(main_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(main_panel
  pluginlib
  rviz_common
  std_srvs
  rclcpp
  rviz_common
  rviz_rendering
  visualization_msgs
  geometry_msgs
)
install(TARGETS main_panel
        EXPORT export_drone_viz
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/dummy_arm_service.py
        DESTINATION lib/${PROJECT_NAME})
# install(FILES icons/classes/DemoPanel.png
#         DESTINATION share/${PROJECT_NAME}/icons/classes
# )
ament_export_include_directories(include)
ament_export_targets(export_drone_viz)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()