#ifndef DRONE_VIZ__CLICKED_POINT_MARKER_HPP_
#define DRONE_VIZ__CLICKED_POINT_MARKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <deque>
#include <std_srvs/srv/trigger.hpp>

class ClickedPointMarker : public rclcpp::Node
{
public:
  ClickedPointMarker();

  void undo_last_marker();

  bool has_last_point() const;

  geometry_msgs::msg::PointStamped get_last_point() const;

private:
  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undo_service_;
  void handle_undo_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool has_point_;
  geometry_msgs::msg::PointStamped last_clicked_point_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_point_service_;


  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  int marker_id_;

  std::deque<int> marker_id_history_;
};

#endif  // DRONE_VIZ__CLICKED_POINT_MARKER_HPP_
