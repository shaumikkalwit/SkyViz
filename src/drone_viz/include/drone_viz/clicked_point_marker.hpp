#ifndef DRONE_VIZ__CLICKED_POINT_MARKER_HPP_
#define DRONE_VIZ__CLICKED_POINT_MARKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

class ClickedPointMarker : public rclcpp::Node
{
public:
  ClickedPointMarker();

private:
  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  int marker_id_;
};

#endif  // DRONE_VIZ__CLICKED_POINT_MARKER_HPP_
