#include <drone_viz/clicked_point_marker.hpp>

ClickedPointMarker::ClickedPointMarker()
: Node("clicked_point_marker"), marker_id_(0)
{
  point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/clicked_point", 10,
    std::bind(&ClickedPointMarker::point_callback, this, std::placeholders::_1)
  );

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
}

void ClickedPointMarker::point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  auto marker = visualization_msgs::msg::Marker();

  marker.header.frame_id = "base_link";
  marker.header.stamp = this->now();
  marker.ns = "clicked_points";
  marker.id = marker_id_++;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position = msg->point;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 10.0;  // 10 meters radius
  marker.scale.y = 10.0;
  marker.scale.z = 10.0;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.frame_locked = true;

  marker.lifetime = rclcpp::Duration::from_seconds(0);  // 0 = forever

  marker_pub_->publish(marker);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClickedPointMarker>());
  rclcpp::shutdown();
  return 0;
}
