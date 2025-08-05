#include <drone_viz/main_panel.hpp>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include "drone_viz_interfaces/msg/command.hpp"
#include "drone_viz_interfaces/srv/flight_service.hpp"
#include <cstdlib>
#include <memory>
#include <chrono>

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
    QPushButton* autonomous_land_button = new QPushButton("Land");

    autonomous_layout->addWidget(autonomous_arm_button);
    autonomous_layout->addWidget(autonomous_takeoff_button);
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
    QObject::connect(autonomous_takeoff_button, &QPushButton::released, this, &MainPanel::landButtonPressed);
    QObject::connect(teleop_land_button, &QPushButton::released, this, &MainPanel::landButtonPressed);
    QObject::connect(autonomous_land_button, &QPushButton::released, this, &MainPanel::landButtonPressed);

    // Gamepad-style directional controls
    auto* direction_layout = new QGridLayout();
    QPushButton* forward_button = new QPushButton("Forward");
    QPushButton* backward_button = new QPushButton("Backward");
    QPushButton* left_button = new QPushButton("Left");
    QPushButton* right_button = new QPushButton("Right");

    // Connect movement buttons
    QObject::connect(forward_button, &QPushButton::released, this, &MainPanel::forwardButtonPressed);
    QObject::connect(backward_button, &QPushButton::released, this, &MainPanel::backwardButtonPressed);
    QObject::connect(left_button, &QPushButton::released, this, &MainPanel::leftButtonPressed);
    QObject::connect(right_button, &QPushButton::released, this, &MainPanel::rightButtonPressed);

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

MainPanel::~MainPanel() {rclcpp::shutdown();}

void MainPanel::onInitialize() {}
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



}  // namespace drone_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(drone_viz::MainPanel, rviz_common::Panel)
