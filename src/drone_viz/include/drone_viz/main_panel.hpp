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
#include <drone_viz_interfaces/msg/command.hpp>
#include <drone_viz_interfaces/srv/flight_service.hpp>
#include <memory>
#include <atomic>



class FlightClientNode {
    private:
        std::atomic<bool> thread_running_state;
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Client<drone_viz_interfaces::srv::FlightService>::SharedPtr client_;
        
        std::shared_ptr<drone_viz_interfaces::srv::FlightService::Request> request_;
    public:
        FlightClientNode() {
            node_ =  rclcpp::Node::make_shared("flight_service_client");
            client_ = node_ -> create_client<drone_viz_interfaces::srv::FlightService>("flight_service");
            request_ = std::make_shared<drone_viz_interfaces::srv::FlightService::Request>();
            thread_running_state = false;
        }
        void threadedRequest(const drone_viz_interfaces::msg::Command & command) {
          if (thread_running_state) {
              RCLCPP_ERROR(node_ -> get_logger(), "Command already pending, request denied.");
              return;
            }
            std::thread t(&FlightClientNode::sendRequest, this, command);
            t.detach();
        }
        void sendRequest(drone_viz_interfaces::msg::Command command) {
            //check if a request is already pending
            
            thread_running_state = true;
            request_ -> command_input = command;
           
            while (!client_ -> wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(node_ -> get_logger(), "Interrupted while waiting for the service. Exiting.");
                } else {
                  RCLCPP_INFO(node_ -> get_logger(), "Service appears to be unavailable... quitting.");
                  thread_running_state = false;
                  return;
                  }
                }
               
            
            auto result = client_ -> async_send_request(request_);

            //wait for result
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(node_);

            while (rclcpp::ok()) {
              executor.spin_some(std::chrono::milliseconds(100));
              if (result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                break;
              }
            }

            if (result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
              auto response = result.get();
              if (response->success) {
                RCLCPP_INFO(node_ -> get_logger(), "SUCCESS!");
                RCLCPP_INFO(node_ -> get_logger(), "Message: %s", response->message.c_str());
              } else {
                RCLCPP_INFO(node_ -> get_logger(), "FAIL!");
                RCLCPP_INFO(node_ -> get_logger(), "Failed, error: %s", response->message.c_str());
              }
            }
            
            thread_running_state = false;
        }
        ~FlightClientNode() {}
};      

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

  // // ROS 2 Node interface
  // std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  // rclcpp::Node::SharedPtr node;

  // // Service client for arming
  // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;

private Q_SLOTS:
  void teleopButtonActivated();
  void autonomousButtonActivated();
  void armButtonPressed();
  void takeoffButtonPressed();
  void landButtonPressed();
  void forwardButtonPressed();
  void backwardButtonPressed();
  void leftButtonPressed();
  void rightButtonPressed();
};
}  // namespace drone_viz



#endif // DRONE_VIZ__MAIN_PANEL_HPP_