#include <drone_viz/waypoint_tool.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/render_window.hpp>
#include <OgreRay.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace drone_viz
{

WaypointTool::WaypointTool()
{
  shortcut_key_ = 'w';  // Optional shortcut in RViz
}

WaypointTool::~WaypointTool() = default;

void WaypointTool::onInitialize()
{
  ros_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  marker_pub_ = ros_node_->create_publisher<visualization_msgs::msg::Marker>(
    "/waypoint_marker", 10);
}

void WaypointTool::activate() {}
void WaypointTool::deactivate() {}

int WaypointTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.leftDown())
  {
    Ogre::Ray mouse_ray = event.viewport->getCamera()->getCameraToViewportRay(
      (float)event.x / event.viewport->getActualWidth(),
      (float)event.y / event.viewport->getActualHeight());

    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0);
    std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(ground_plane);
    if (intersection.first)
    {
      Ogre::Vector3 pos = mouse_ray.getPoint(intersection.second);

      clicked_point_.header.frame_id = "map";  // Change if needed
      clicked_point_.header.stamp = ros_node_->get_clock()->now();
      clicked_point_.point.x = pos.x;
      clicked_point_.point.y = pos.y;
      clicked_point_.point.z = 0.0;

      point_ready_ = true;

      publishMarker(clicked_point_);
    }
  }

  return Render;
}

void WaypointTool::publishMarker(const geometry_msgs::msg::PointStamped& point)
{
  visualization_msgs::msg::Marker marker;
  marker.header = point.header;
  marker.ns = "waypoint";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = point.point;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.8;
  marker.lifetime = rclcpp::Duration::from_seconds(10.0);

  marker_pub_->publish(marker);
}

geometry_msgs::msg::PointStamped WaypointTool::getLastClickedPoint() const
{
  return clicked_point_;
}

bool WaypointTool::hasNewPoint() const
{
  return point_ready_;
}

} // namespace drone_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(drone_viz::WaypointTool, rviz_common::Tool)
