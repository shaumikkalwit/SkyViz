#pragma once

#include <rviz_common/tool.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_rendering/render_window.hpp>

namespace drone_viz
{

class WaypointTool : public rviz_common::Tool
{
public:
  WaypointTool();
  ~WaypointTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

  geometry_msgs::msg::PointStamped getLastClickedPoint() const;
  bool hasNewPoint() const;

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  geometry_msgs::msg::PointStamped clicked_point_;
  bool point_ready_ = false;

  void publishMarker(const geometry_msgs::msg::PointStamped& point);
};

} // namespace drone_viz
