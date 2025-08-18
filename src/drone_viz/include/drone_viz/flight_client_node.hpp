#ifndef FLIGHT_CLIENT_NODE_HPP
#define FLIGHT_CLIENT_NODE_HPP

#include <memory>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <drone_viz_interfaces/msg/command.hpp>
#include <drone_viz_interfaces/srv/flight_service.hpp>


class FlightClientNode {
  private:
      std::atomic<bool> thread_running_state;
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Client<drone_viz_interfaces::srv::FlightService>::SharedPtr client_;
      
      std::shared_ptr<drone_viz_interfaces::srv::FlightService::Request> request_;
  public:
      FlightClientNode(const std::string &service_name) {
          node_ =  rclcpp::Node::make_shared("flight_service_client");
          client_ = node_->create_client<drone_viz_interfaces::srv::FlightService>(service_name);
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
            }
            else {
              RCLCPP_INFO(node_ -> get_logger(), "FAIL!");
              RCLCPP_INFO(node_ -> get_logger(), "Failed, error: %s", response->message.c_str());
            }
          }
          
          thread_running_state = false;
      }
      ~FlightClientNode() {}
};

#endif // FLIGHT_CLIENT_NODE_HPP