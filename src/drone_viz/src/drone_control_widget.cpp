// // drone_control_widget.cpp
// #include "drone_viz/drone_control_widget.hpp"
// #include "drone_viz/main_panel.hpp" // We need the full definition of MainPanel to connect to its slots

// #include <QVBoxLayout>
// #include <QGridLayout>
// #include <QPushButton>
// #include <QLabel>
// #include <QStackedLayout>

// DroneControlWidget::DroneControlWidget(drone_viz::MainPanel* parent_panel) : QWidget(parent_panel)
// {
//     // --- Create the three pages (main, autonomous, teleop) ---
//     main_widget_ = new QWidget();
//     auto* main_layout = new QVBoxLayout(main_widget_);
//     QPushButton* autonomous_button = new QPushButton("Autonomous Control");
//     QPushButton* teleop_button = new QPushButton("Teleoperation Control");
//     main_layout->addWidget(autonomous_button);
//     main_layout->addWidget(teleop_button);

//     autonomous_widget_ = new QWidget();
//     auto* autonomous_layout = new QVBoxLayout(autonomous_widget_);
//     autonomous_layout->addWidget(new QLabel("Autonomous Mode"));

//     teleop_widget_ = new QWidget();
//     auto* teleop_layout = new QVBoxLayout(teleop_widget_);
//     teleop_layout->addWidget(new QLabel("Teleoperation Mode"));

//     // --- Create the stacked layout to switch between pages ---
//     stacked_layout_ = new QStackedLayout(this);
//     stacked_layout_->addWidget(main_widget_);
//     stacked_layout_->addWidget(teleop_widget_);
//     stacked_layout_->addWidget(autonomous_widget_);
//     setLayout(stacked_layout_);

//     // --- Connect page-switching buttons to change pages within this widget ---
//     connect(autonomous_button, &QPushButton::released, [this]() {
//         stacked_layout_->setCurrentWidget(autonomous_widget_);
//     });
//     connect(teleop_button, &QPushButton::released, [this]() {
//         stacked_layout_->setCurrentWidget(teleop_widget_);
//     });

//     // --- Create all control buttons for the "Autonomous" page ---
//     QPushButton* autonomous_arm_button = new QPushButton("Arm");
//     QPushButton* autonomous_takeoff_button = new QPushButton("Takeoff");
//     QPushButton* confirm_button = new QPushButton("Confirm Waypoint");
//     QPushButton* undo_button = new QPushButton("Undo Waypoint");
//     QPushButton* autonomous_land_button = new QPushButton("Land");
//     QPushButton* autonomous_back_button = new QPushButton("Back");
//     autonomous_layout->addWidget(autonomous_arm_button);
//     autonomous_layout->addWidget(autonomous_takeoff_button);
//     autonomous_layout->addWidget(confirm_button);
//     autonomous_layout->addWidget(undo_button);
//     autonomous_layout->addWidget(autonomous_land_button);
//     autonomous_layout->addWidget(autonomous_back_button);

//     // --- Create all control buttons for the "Teleoperation" page ---
//     QPushButton* teleop_arm_button = new QPushButton("Arm");
//     QPushButton* teleop_takeoff_button = new QPushButton("Takeoff");
//     QPushButton* teleop_land_button = new QPushButton("Land");
//     QPushButton* teleop_back_button = new QPushButton("Back");
//     teleop_layout->addWidget(teleop_arm_button);
//     teleop_layout->addWidget(teleop_takeoff_button);
//     teleop_layout->addWidget(teleop_land_button);

//     // Directional buttons for Teleop
//     auto* direction_layout = new QGridLayout();
//     QPushButton* forward_button = new QPushButton("Forward");
//     QPushButton* backward_button = new QPushButton("Backward");
//     QPushButton* left_button = new QPushButton("Left");
//     QPushButton* right_button = new QPushButton("Right");
//     direction_layout->addWidget(forward_button, 0, 1);
//     direction_layout->addWidget(left_button, 1, 0);
//     direction_layout->addWidget(right_button, 1, 2);
//     direction_layout->addWidget(backward_button, 2, 1);
//     teleop_layout->addLayout(direction_layout);
//     teleop_layout->addWidget(teleop_back_button);

//     // --- Connect all buttons to the MainPanel's slots (The "Pilot") ---
//     // This is the most important part: this widget just sends signals up to the MainPanel.
//     connect(autonomous_arm_button, &QPushButton::released, parent_panel, &drone_viz::MainPanel::armButtonPressed);
//     connect(teleop_arm_button, &QPushButton::released, parent_panel, &drone_viz::MainPanel::armButtonPressed);
//     connect(autonomous_takeoff_button, &QPushButton::released, parent_panel, &drone_viz::MainPanel::takeoffButtonPressed);
//     connect(teleop_takeoff_button, &QPushButton::released, parent_panel, &drone_viz::MainPanel::takeoffButtonPressed);
//     connect(autonomous_land_button, &QPushButton::released, parent_panel, &drone_viz::MainPanel::landButtonPressed);
//     connect(teleop_land_button, &QPushButton::released, parent_panel, &drone_viz::MainPanel::landButtonPressed);
//     connect(confirm_button, &QPushButton::released, parent_panel, &drone_viz::MainPanel::confirmWaypointButtonPressed);
//     connect(undo_button, &QPushButton::released, parent_panel, &drone_viz::MainPanel::undoButtonPressed);
//     connect(forward_button, &QPushButton::released, parent_panel, &drone_viz::MainPanel::forwardButtonPressed);
//     connect(backward_button, &QPushButton::released, parent_panel, &drone_viz::MainPanel::backwardButtonPressed);
//     connect(left_button, &QPushButton::released, parent_panel, &drone_viz::MainPanel::leftButtonPressed);
//     connect(right_button, &QPushButton::released, parent_panel, &drone_viz::MainPanel::rightButtonPressed);

//     // Connect the local "Back" buttons to switch back to the main page of this widget
//     auto go_back_to_main_page = [this](){ stacked_layout_->setCurrentWidget(main_widget_); };
//     connect(autonomous_back_button, &QPushButton::released, go_back_to_main_page);
//     connect(teleop_back_button, &QPushButton::released, go_back_to_main_page);
// }

#include "drone_viz/drone_control_widget.hpp"
#include "drone_viz/main_panel.hpp" // Required for the constructor
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QStackedLayout>

DroneControlWidget::DroneControlWidget(drone_viz::MainPanel* parent_panel) : QWidget(parent_panel)
{
    // --- Create the UI pages ---
    main_widget_ = new QWidget();
    auto* main_layout = new QVBoxLayout(main_widget_);
    QPushButton* autonomous_button = new QPushButton("Autonomous Control");
    QPushButton* teleop_button = new QPushButton("Teleoperation Control");
    main_layout->addWidget(autonomous_button);
    main_layout->addWidget(teleop_button);

    autonomous_widget_ = new QWidget();
    auto* autonomous_layout = new QVBoxLayout(autonomous_widget_);
    autonomous_layout->addWidget(new QLabel("Autonomous Mode"));
    
    // --- Create the stacked layout to switch between pages ---
    stacked_layout_ = new QStackedLayout(this);
    stacked_layout_->addWidget(main_widget_);
    stacked_layout_->addWidget(autonomous_widget_); // Simplified for now
    setLayout(stacked_layout_);
    
    // --- Connect page-switching buttons ---
    connect(autonomous_button, &QPushButton::released, [this](){ stacked_layout_->setCurrentWidget(autonomous_widget_); });
    // connect(teleop_button, &QPushButton::released, [this](){ stacked_layout_->setCurrentWidget(teleop_widget_); });

    // --- Create placeholder buttons ---
    autonomous_layout->addWidget(new QPushButton("Arm"));
    autonomous_layout->addWidget(new QPushButton("Takeoff"));
    autonomous_layout->addWidget(new QPushButton("Land"));

    // NO backend connections yet. The buttons will appear, but do nothing.
}