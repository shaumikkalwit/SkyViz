// drone.hpp
#ifndef DRONE_HPP
#define DRONE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // Use PoseStamped for frame and timestamp info
#include <memory>
#include <string>

// Forward-declare to avoid including the full header here
class FlightClientNode; 
namespace drone_viz_interfaces { namespace msg { struct Command; } }

namespace drone_viz
{

class Drone
{
public:
    // The constructor takes all necessary configuration and a shared node pointer
    Drone(rclcpp::Node::SharedPtr node,
          const std::string& name, 
          const std::string& pose_topic, 
          const std::string& flight_service_name);
    
    ~Drone() = default;
    
    // --- Getters ---
    const std::string& getName() const { return name_; }
    const geometry_msgs::msg::PoseStamped& getCurrentPose() const { return current_pose_; }
    bool isPoseValid() const { return pose_received_; }
    
    // --- Actions ---
    // A clean interface to send any command to this specific drone
    void sendFlightCommand(const drone_viz_interfaces::msg::Command &command);
    
private:
    // Callback for the pose subscriber
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // --- Member Variables ---
    std::string name_;
    rclcpp::Node::SharedPtr node_;
    
    // ROS Communication
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    std::shared_ptr<FlightClientNode> flight_client_;
    
    // Drone State
    geometry_msgs::msg::PoseStamped current_pose_;
    bool pose_received_;
};

} // namespace drone_viz

#endif // DRONE_HPP