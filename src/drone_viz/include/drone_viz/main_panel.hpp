#ifndef DRONE_VIZ__MAIN_PANEL_HPP_
#define DRONE_VIZ__MAIN_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <QLabel>
#include <QPushButton>
#include <QStackedLayout> // Qt layout manager that stacks multiple widgets on top of each other, but shows only one at a time
#include <QWidget>
#include <QTabWidget>


#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <drone_viz/clicked_point_marker.hpp>

#include <drone_viz/flight_client_node.hpp>


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
  bool absolutecommand;
  drone_viz_interfaces::msg::Command message;
  std::shared_ptr<FlightClientNode> flightclient = std::make_shared<FlightClientNode>();
  double increment = 0.5;

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
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<ClickedPointMarker> clicked_point_marker_node_;

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr get_point_client;

  QTabWidget* tab_widget_;
  std::string current_drone_name_;

private Q_SLOTS:
  void onTabChanged(int index);

  void confirmWaypointButtonPressed();
  void teleopButtonActivated();
  void autonomousButtonActivated();
  void armButtonPressed();
  void takeoffButtonPressed();
  void landButtonPressed();
  void forwardButtonPressed();
  void backwardButtonPressed();
  void leftButtonPressed();
  void rightButtonPressed();
  void undoButtonPressed();
};
}  // namespace drone_viz

#endif // DRONE_VIZ__MAIN_PANEL_HPP_