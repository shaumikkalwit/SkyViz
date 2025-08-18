#include <drone_viz/clicked_point_marker.hpp>

// stores points clicked 

ClickedPointMarker::ClickedPointMarker()
: Node("clicked_point_marker"), marker_id_(0)
{
  point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/clicked_point", 10,
    std::bind(&ClickedPointMarker::point_callback, this, std::placeholders::_1)
  );

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

  undo_service_ = this->create_service<std_srvs::srv::Trigger>(
    "undo_marker",
    std::bind(&ClickedPointMarker::handle_undo_request, this, std::placeholders::_1, std::placeholders::_2)
  );

  get_point_service_ = this->create_service<std_srvs::srv::Trigger>(
    "get_last_point",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      if (has_point_) {
        RCLCPP_INFO(this->get_logger(), "Service call: last point is ready");
        res->success = true;
        res->message = std::to_string(last_clicked_point_.point.x) + ", " +
                      std::to_string(last_clicked_point_.point.y) + ", " +
                      std::to_string(last_clicked_point_.point.z);
      } else {
        RCLCPP_WARN(this->get_logger(), "No point clicked yet.");
        res->success = false;
        res->message = "No point available";
      }
    }
  );

}

bool ClickedPointMarker::has_last_point() const
{
  return has_point_;
}

geometry_msgs::msg::PointStamped ClickedPointMarker::get_last_point() const
{
  return last_clicked_point_;
}


void ClickedPointMarker::point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  auto marker = visualization_msgs::msg::Marker();

  marker.header.frame_id = "arena";
  marker.header.stamp = this->now();
  marker.ns = "clicked_points";
  marker.id = marker_id_++;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position = msg->point;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.frame_locked = true;

  marker.lifetime = rclcpp::Duration::from_seconds(0);  // 0 = forever
  last_clicked_point_ = *msg;
  has_point_ = true;


  marker_pub_->publish(marker);

  // Store marker ID for possible undo
  marker_id_history_.push_back(marker.id);
}

void ClickedPointMarker::undo_last_marker()
{
  if (!marker_id_history_.empty()) {
    // Get the last marker ID
    int last_marker_id = marker_id_history_.back();
    marker_id_history_.pop_back();

    auto delete_marker = visualization_msgs::msg::Marker();
    delete_marker.header.frame_id = "arena";
    delete_marker.header.stamp = this->now();
    delete_marker.ns = "clicked_points";
    delete_marker.id = last_marker_id;
    delete_marker.action = visualization_msgs::msg::Marker::DELETE;

    marker_pub_->publish(delete_marker);
  }
}

void ClickedPointMarker::handle_undo_request(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!marker_id_history_.empty()) {
    undo_last_marker();
    response->success = true;
    response->message = "Last marker undone.";
  } else {
    response->success = false;
    response->message = "No marker to undo.";
  }
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClickedPointMarker>());
  rclcpp::shutdown();
  return 0;
}
