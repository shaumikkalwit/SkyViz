#include <drone_viz/main_panel.hpp>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <drone_viz/clicked_point_marker.hpp>

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

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;

    // Create the autonomous widget and its layout
    autonomous_widget_ = new QWidget();
    auto* autonomous_layout = new QVBoxLayout(autonomous_widget_);
    autonomous_layout->addWidget(new QLabel("Autonomous Mode"));

    // --- Autonomous mode buttons ---
    QPushButton* autonomous_arm_button = new QPushButton("Arm");
    QPushButton* autonomous_takeoff_button = new QPushButton("Takeoff");
    QPushButton* autonomous_land_button = new QPushButton("Land");
    QPushButton* undo_button = new QPushButton("Undo");

    autonomous_layout->addWidget(autonomous_arm_button);
    autonomous_layout->addWidget(autonomous_takeoff_button);
    autonomous_layout->addWidget(autonomous_land_button);
    autonomous_layout->addWidget(undo_button);

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

    // // --- Connect both arm buttons ---
    // QObject::connect(autonomous_arm_button, &QPushButton::released, this, &MainPanel::armButtonPressed);
    // QObject::connect(teleop_arm_button, &QPushButton::released, this, &MainPanel::armButtonPressed);

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

    QObject::connect(undo_button, &QPushButton::released, this, &MainPanel::undoButtonPressed);

    clicked_point_marker_node_ = std::make_shared<ClickedPointMarker>();

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
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

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
    // auto message = std_msgs::msg::String();
    // message.data = "Button clicked!";
    // publisher_->publish(message);

    stacked_layout_->setCurrentWidget(teleop_widget_);
}

void MainPanel::autonomousButtonActivated()
{
    // auto message = std_msgs::msg::String();
    // message.data = "Button clicked!";
    // publisher_->publish(message);

    stacked_layout_->setCurrentWidget(autonomous_widget_);
}

void MainPanel::undoButtonPressed()
{
    // Call the undo function in your ClickedPointMarker node
    if (clicked_point_marker_node_)
    {
        clicked_point_marker_node_->undo_last_marker();
    }
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