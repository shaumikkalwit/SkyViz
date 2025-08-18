// drone.cpp
#include "drone_viz/drone.hpp"
#include "drone_viz/flight_client_node.hpp"
#include "drone_viz_interfaces/msg/command.hpp"

namespace drone_viz
{

// The constructor implementation
Drone::Drone(rclcpp::Node::SharedPtr node,
         const std::string& name, 
         const std::string& pose_topic, 
         const std::string& flight_service_name)
    // Use an initializer list for efficiency and correctness
    : name_(name),
      node_(node),
      pose_received_(false)
{
    // Create the subscriber for the drone's pose topic
    pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic,
        10, // QoS setting: history depth
        std::bind(&Drone::poseCallback, this, std::placeholders::_1)
    );

    // Each drone gets its own dedicated flight client
    flight_client_ = std::make_shared<FlightClientNode>(flight_service_name);

    RCLCPP_INFO(node_->get_logger(), "Drone '%s' initialized and subscribed to '%s'.", name_.c_str(), pose_topic.c_str());
}

// This callback is triggered every time a new pose message arrives
void Drone::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_pose_ = *msg; // Update the drone's current pose
    pose_received_ = true; // Mark that we have received at least one valid pose
}

// This method provides a clean way for the MainPanel to issue commands
void Drone::sendFlightCommand(const drone_viz_interfaces::msg::Command &command)
{
    // Check if the flight client was successfully created before using it
    if (flight_client_) {
        flight_client_->threadedRequest(command);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Flight client for drone '%s' is not available.", name_.c_str());
    }
}

} // namespace drone_viz