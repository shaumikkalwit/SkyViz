#include <drone_viz/waypoint_tool.hpp>
#include <rclcpp/rclcpp.hpp>

namespace drone_viz
{

WaypointTool::WaypointTool()
{
  shortcut_key_ = 'w';
}

void WaypointTool::onInitialize()
{
  setName("Waypoint Tool");
}

void WaypointTool::activate()
{
  // Optional: code to run when the tool is activated
}

void WaypointTool::deactivate()
{
  // Optional: code to run when the tool is deactivated
}

}  // namespace drone_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(drone_viz::WaypointTool, rviz_common::Tool)
