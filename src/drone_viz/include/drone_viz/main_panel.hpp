#ifndef DRONE_VIZ__MAIN_PANEL_HPP_
#define DRONE_VIZ__MAIN_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <QLabel>
#include <QPushButton>
#include <QStackedLayout> // Qt layout manager that stacks multiple widgets on top of each other, but shows only one at a time
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <drone_viz/clicked_point_marker.hpp>


namespace drone_viz
{
class MainPanel
  : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit MainPanel(QWidget * parent = 0);
  ~MainPanel() override;

  void onInitialize() override;

protected:
  // std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  // void topicCallback(const std_msgs::msg::String & msg);

  QLabel* label_;
  QPushButton* teleop_button_;
  QPushButton* autonomous_button_;

  QStackedLayout* stacked_layout_;
  QWidget* main_widget_;
  QWidget* teleop_widget_;
  QWidget* autonomous_widget_;

  QPushButton* forward_button_;
  QPushButton* backward_button_;
  QPushButton* left_button_;
  QPushButton* right_button_;

  // ROS 2 Node interface
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Node::SharedPtr node;

  std::shared_ptr<ClickedPointMarker> clicked_point_marker_node_;

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undo_client_;

private Q_SLOTS:
  void confirmButtonPressed();
  void teleopButtonActivated();
  void autonomousButtonActivated();
  void armButtonPressed();
  void undoButtonPressed();
};
}  // namespace drone_viz

#endif // DRONE_VIZ__MAIN_PANEL_HPP_