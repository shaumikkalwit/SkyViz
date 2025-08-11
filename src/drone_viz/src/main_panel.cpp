#include <drone_viz/main_panel.hpp>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <drone_viz/clicked_point_marker.hpp>
#include <drone_viz/flight_client_node.hpp>
#include "drone_viz_interfaces/msg/command.hpp"
#include "drone_viz_interfaces/srv/flight_service.hpp"
#include <cstdlib>
#include <memory>
#include <chrono>
#include <sstream> // Required for std::stringstream
#include <geometry_msgs/msg/point.hpp> // Required for geometry_msgs::msg::Point


// #include <rviz_common/ros_node_abstraction_iface.hpp>


namespace  drone_viz
{
MainPanel::MainPanel(QWidget* parent) : Panel(parent)
{
    main_widget_ = new QWidget();
    auto* main_layout = new QVBoxLayout(main_widget_);  // layout for main_widget_
    label_ = new QLabel("[no data]");
    autonomous_button_ = new QPushButton("Autonomous Control");
    teleop_button_ = new QPushButton("Teleoperation Control");
    main_layout->addWidget(label_);
    main_layout->addWidget(autonomous_button_);
    main_layout->addWidget(teleop_button_);

    QObject::connect(autonomous_button_, &QPushButton::released, this, &MainPanel::autonomousButtonActivated);
    QObject::connect(teleop_button_, &QPushButton::released, this, &MainPanel::teleopButtonActivated);

    // Create the autonomous widget and its layout
    autonomous_widget_ = new QWidget();
    auto* autonomous_layout = new QVBoxLayout(autonomous_widget_);
    autonomous_layout->addWidget(new QLabel("Autonomous Mode"));

    // --- Autonomous mode buttons ---
    QPushButton* autonomous_arm_button = new QPushButton("Arm");
    QPushButton* autonomous_takeoff_button = new QPushButton("Takeoff");
    QPushButton* confirm_button = new QPushButton("Confirm");
    QPushButton* undo_button = new QPushButton("Undo");
    QPushButton* autonomous_land_button = new QPushButton("Land");

    autonomous_layout->addWidget(autonomous_arm_button);
    autonomous_layout->addWidget(autonomous_takeoff_button);
    autonomous_layout->addWidget(confirm_button);
    autonomous_layout->addWidget(undo_button);
    autonomous_layout->addWidget(autonomous_land_button);

    // Create the teleop widget and set its layout
    teleop_widget_ = new QWidget();
    auto* teleop_layout = new QVBoxLayout(teleop_widget_);
    teleop_layout->addWidget(new QLabel("Teleoperation Mode"));

    // --- Teleop mode buttons ---
    QPushButton* teleop_arm_button = new QPushButton("Arm");
    QPushButton* teleop_takeoff_button = new QPushButton("Takeoff");
    QPushButton* teleop_land_button = new QPushButton("Land");

    teleop_layout->addWidget(teleop_arm_button);
    teleop_layout->addWidget(teleop_takeoff_button);
    teleop_layout->addWidget(teleop_land_button);

     // --- Connect arm, takeoff and land buttons ---
    QObject::connect(autonomous_arm_button, &QPushButton::released, this, &MainPanel::armButtonPressed);
    QObject::connect(teleop_arm_button, &QPushButton::released, this, &MainPanel::armButtonPressed);
    QObject::connect(teleop_takeoff_button, &QPushButton::released, this, &MainPanel::takeoffButtonPressed);
    QObject::connect(autonomous_takeoff_button, &QPushButton::released, this, &MainPanel::takeoffButtonPressed);
    QObject::connect(teleop_land_button, &QPushButton::released, this, &MainPanel::landButtonPressed);
    QObject::connect(autonomous_land_button, &QPushButton::released, this, &MainPanel::landButtonPressed);

    // Gamepad-style directional controls
    auto* direction_layout = new QGridLayout();
    QPushButton* forward_button = new QPushButton("Forward");
    QPushButton* backward_button = new QPushButton("Backward");
    QPushButton* left_button = new QPushButton("Left");
    QPushButton* right_button = new QPushButton("Right");

    direction_layout->addWidget(forward_button, 0, 1);  // Top center
    direction_layout->addWidget(left_button, 1, 0);     // Middle left
    direction_layout->addWidget(right_button, 1, 2);    // Middle right
    direction_layout->addWidget(backward_button, 2, 1);     // Bottom center

    // Optional: Add a container widget to make it tidy
    QWidget* direction_widget = new QWidget();
    direction_widget->setLayout(direction_layout);
    teleop_layout->addWidget(direction_widget);

    // Teleop back button
    QPushButton* teleop_back_button = new QPushButton("Back");
    teleop_layout->addWidget(teleop_back_button);
    QObject::connect(teleop_back_button, &QPushButton::released, [this]() {
        stacked_layout_->setCurrentWidget(main_widget_);
    });

    // Autonomous back button
    QPushButton* autonomous_back_button = new QPushButton("Back");
    autonomous_layout->addWidget(autonomous_back_button);
    QObject::connect(autonomous_back_button, &QPushButton::released, [this]() {
        stacked_layout_->setCurrentWidget(main_widget_);
    });

    QObject::connect(confirm_button, &QPushButton::released, this, &MainPanel::confirmWaypointButtonPressed);

    QObject::connect(undo_button, &QPushButton::released, this, &MainPanel::undoButtonPressed);

    // Create the stacked layout and add the two widgets
    stacked_layout_ = new QStackedLayout(this);  // Set this as parent
    stacked_layout_->addWidget(main_widget_);
    stacked_layout_->addWidget(teleop_widget_);
    stacked_layout_->addWidget(autonomous_widget_);

    // Set the stacked layout as the panel's layout
    setLayout(stacked_layout_);

    // Show the main widget by default
    stacked_layout_->setCurrentWidget(main_widget_);
}

MainPanel::~MainPanel() = default;

void MainPanel::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)gi
  node = node_ptr_->get_raw_node();
  undo_client_ = node->create_client<std_srvs::srv::Trigger>("undo_marker");

  get_point_client = node->create_client<std_srvs::srv::Trigger>("get_last_point");

  clicked_point_marker_node_ = std::make_shared<ClickedPointMarker>();


  // // Create a String publisher for the output
  // publisher_ = node->create_publisher<std_msgs::msg::String>("/output", 10);

  // // Create a String subscription and bind it to the topicCallback inside this class.
  // subscription_ = node->create_subscription<std_msgs::msg::String>("/input", 10, std::bind(&MainPanel::topicCallback, this, std::placeholders::_1));
}

// // When the subscriber gets a message, this callback is triggered,
// // and then we copy its data into the widget's label
// void MainPanel::topicCallback(const std_msgs::msg::String & msg)
// {
//   label_->setText(QString(msg.data.c_str()));
// }

// When the widget's button is pressed, this callback is triggered,
// and then we publish a new message on our topic.
void MainPanel::teleopButtonActivated()
{  
    absolutecommand = false;
    stacked_layout_->setCurrentWidget(teleop_widget_);
}

void MainPanel::autonomousButtonActivated()
{
    absolutecommand = true;
    stacked_layout_->setCurrentWidget(autonomous_widget_);
}

void MainPanel::armButtonPressed()
{
    message.absolute = absolutecommand;
    message.comtype = 'a';
    flightclient -> threadedRequest(message);
    
}

void MainPanel::takeoffButtonPressed()
{
    message.comtype = 't';
    flightclient -> threadedRequest(message);
}

void MainPanel::landButtonPressed()
{
    message.comtype = 'l';
    flightclient -> threadedRequest(message);
}

void MainPanel::leftButtonPressed()
{
    message.comtype = 'm';
    message.moveto.x = 0;
    message.moveto.y = -increment;
    message.moveto.z = 0;

    flightclient -> threadedRequest(message);
}

void MainPanel::rightButtonPressed()
{
    message.comtype = 'm';
    message.moveto.x = 0;
    message.moveto.y = increment;
    message.moveto.z = 0;

    flightclient -> threadedRequest(message);
}

void MainPanel::forwardButtonPressed()
{
    message.comtype = 'm';
    message.moveto.x = increment;
    message.moveto.y = 0;
    message.moveto.z = 0;

    flightclient -> threadedRequest(message);
}

void MainPanel::backwardButtonPressed()
{
    message.comtype = 'm';
    message.moveto.x = -increment;
    message.moveto.y = 0;
    message.moveto.z = 0;

    flightclient -> threadedRequest(message);
}

void MainPanel::confirmWaypointButtonPressed()
{
  // //  client for the "get_last_point" service
  // auto get_point_client = node->create_client<std_srvs::srv::Trigger>("get_last_point");

  // // Check if the service is available
  // if (!get_point_client->wait_for_service(std::chrono::seconds(1))) {
  //   RCLCPP_ERROR(node->get_logger(), "Service 'get_last_point' not available. Is the clicked_point_marker node running?");
  //   return;
  // }

  // // create the request and send it asynchronously
  // auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  // auto future_result = get_point_client->async_send_request(request,
  //   // define the callback function that will run when the service responds
  //   [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
  //   {
  //     auto response = future.get();
  //     if (response && response->success) {
  //       RCLCPP_INFO(node->get_logger(), "Received point from service: %s", response->message.c_str());

  //       geometry_msgs::msg::Point target_point;
  //       std::stringstream ss(response->message);
  //       char comma; // To consume the commas
  //       ss >> target_point.x >> comma >> target_point.y >> comma >> target_point.z;

  //       target_point.z = 1.0;

  //       // command message for the drone.
  //       message.comtype = "m"; // 'm' for moveto
  //       message.moveto = target_point;
  //       RCLCPP_INFO(node->get_logger(), "CHECK IF YOU GET HERE FOR USER WYAPOINT");
  //       // message.absolute = false; 

  //       // send the command to the flight client
  //       flightclient->threadedRequest(message);
  //       RCLCPP_INFO(node->get_logger(), "Waypoint command sent to flight client.");

  //     } else {
  //       RCLCPP_WARN(node->get_logger(), "Failed to get point: %s", response->message.c_str());
  //     }
  //   });
  if (!this->get_point_client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node->get_logger(), "Service 'get_last_point' not available. Is the clicked_point_marker node running?");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  
  // Use the member variable to send the request. Because the client is a member
  // of the class, it will persist and be able to receive the response.
  this->get_point_client->async_send_request(request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
    {
      // This whole section is the callback that runs when the response arrives.
      auto response = future.get();

      // Safely check if the response is valid and if the service call was successful.
      if (response && response->success) {
        RCLCPP_INFO(node->get_logger(), "Received point from service: %s", response->message.c_str());

        geometry_msgs::msg::Point target_point;
        
        // Use a stringstream to parse the x,y,z coordinates from the response message string.
        std::stringstream ss(response->message);
        char comma; // Used to consume the commas in the string.
        ss >> target_point.x >> comma >> target_point.y >> comma >> target_point.z;

        // Optionally, override a coordinate like Z to set a fixed flight altitude.
        target_point.z = 1.0;

        // Prepare the command message for the drone.
        message.comtype = 'm'; // 'm' for moveto
        message.moveto = target_point;
        // message.absolute = true; // Clicked points are absolute coordinates.

        // Send the command to the flight client.
        flightclient->threadedRequest(message);
        RCLCPP_INFO(node->get_logger(), "Waypoint command sent to flight client.");

      } else {
        // This block will run if the service call fails or returns success=false.
        RCLCPP_WARN(node->get_logger(), "Failed to get point: %s", response ? response->message.c_str() : "Service call failed");
      }
    });
}


void MainPanel::undoButtonPressed()
{
  if (!undo_client_ || !undo_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_ptr_->get_raw_node()->get_logger(), "Undo service not available");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto future = undo_client_->async_send_request(request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
      auto response = result.get();
      if (response->success) {
        RCLCPP_INFO(node_ptr_->get_raw_node()->get_logger(), "Undo successful: %s", response->message.c_str());
      } else {
        RCLCPP_WARN(node_ptr_->get_raw_node()->get_logger(), "Undo failed: %s", response->message.c_str());
      }
    }
  );
}


// void MainPanel::armButtonPressed()
// {
//     if (!arm_client_ || !arm_client_->wait_for_service(std::chrono::seconds(1))) {
//             RCLCPP_WARN(node->get_logger(), "Arm service not available");
//             return;
//         }

//         auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
//         request->data = true;

//         auto future = arm_client_->async_send_request(request,
//             [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result) {
//                 auto response = result.get();
//                 if (response->success) {
//                     RCLCPP_INFO(node->get_logger(), "Drone armed: %s", response->message.c_str());
//                 } else {
//                     RCLCPP_WARN(node->get_logger(), "Failed to arm: %s", response->message.c_str());
//                 }
//             }
//         );
// }

}  // namespace drone_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(drone_viz::MainPanel, rviz_common::Panel)