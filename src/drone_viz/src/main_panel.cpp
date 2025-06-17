#include <drone_viz/main_panel.hpp>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

namespace  drone_viz
{
MainPanel::MainPanel(QWidget* parent) : Panel(parent)
{
    // main_widget_ = new QWidget();
    // // Create a label and a button, displayed vertically (the V in VBox means vertical)
    // auto* main_layout = new QVBoxLayout(this);
    // // Create a button and a label for the button
    // label_ = new QLabel("[no data]");
    // teleop_button_ = new QPushButton("GO!");
    // // Add those elements to the GUI layout
    // main_layout->addWidget(label_);
    // main_layout->addWidget(teleop_button_);

    // // Connect the event of when the button is released to our callback,
    // // so pressing the button results in the buttonActivated callback being called.
    // QObject::connect(teleop_button_, &QPushButton::released, this, &MainPanel::buttonActivated);

    // // ----------- TELEOP VIEW -----------
    // teleop_widget_ = new QWidget();
    // auto* teleop_layout = new QVBoxLayout(teleop_widget_);
    // teleop_layout->addWidget(new QLabel("Teleoperation Mode"));

    // QPushButton* arm_button = new QPushButton("Arm");
    // teleop_layout->addWidget(arm_button);

    // QPushButton* takeoff_button = new QPushButton("Takeoff");
    // teleop_layout->addWidget(takeoff_button);

    // QPushButton* land_button = new QPushButton("Land");
    // teleop_layout->addWidget(land_button);

    // QPushButton* back_button = new QPushButton("Back");
    // teleop_layout->addWidget(back_button);
    // QObject::connect(back_button, &QPushButton::released, [this]() {
    //     stacked_layout_->setCurrentWidget(main_widget_);
    // });

    // // ----------- STACK LAYOUT SETUP -----------
    // stacked_layout_ = new QStackedLayout();
    // stacked_layout_->addWidget(main_widget_);
    // stacked_layout_->addWidget(teleop_widget_);
    // stacked_layout_->setCurrentWidget(main_widget_);

    // // if (this->layout())
    // // {
    // //     delete this->layout();  // Remove old layout before setting new one
    // // }
    // setLayout(stacked_layout_);
    // Create the main widget and set its layout
    main_widget_ = new QWidget();
    auto* main_layout = new QVBoxLayout(main_widget_);  // layout for main_widget_
    label_ = new QLabel("[no data]");
    teleop_button_ = new QPushButton("Teleoperation Control");
    main_layout->addWidget(label_);
    main_layout->addWidget(teleop_button_);
    QObject::connect(teleop_button_, &QPushButton::released, this, &MainPanel::buttonActivated);

    // Create the teleop widget and set its layout
    teleop_widget_ = new QWidget();
    auto* teleop_layout = new QVBoxLayout(teleop_widget_);
    teleop_layout->addWidget(new QLabel("Teleoperation Mode"));

    QPushButton* arm_button = new QPushButton("Arm");
    teleop_layout->addWidget(arm_button);

    QPushButton* takeoff_button = new QPushButton("Takeoff");
    teleop_layout->addWidget(takeoff_button);

    QPushButton* land_button = new QPushButton("Land");
    teleop_layout->addWidget(land_button);

    QPushButton* back_button = new QPushButton("Back");
    teleop_layout->addWidget(back_button);
    QObject::connect(back_button, &QPushButton::released, [this]() {
        stacked_layout_->setCurrentWidget(main_widget_);
    });

    // Create the stacked layout and add the two widgets
    stacked_layout_ = new QStackedLayout(this);  // Set this as parent
    stacked_layout_->addWidget(main_widget_);
    stacked_layout_->addWidget(teleop_widget_);

    // Set the stacked layout as the panel's layout
    setLayout(stacked_layout_);

    // Show the main widget by default
    stacked_layout_->setCurrentWidget(main_widget_);
}

MainPanel::~MainPanel() = default;

void MainPanel::onInitialize()
{
  // // Access the abstract ROS Node and
  // // in the process lock it for exclusive use until the method is done.
  // node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // // (as per normal rclcpp code)gi
  // rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

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
void MainPanel::buttonActivated()
{
  // auto message = std_msgs::msg::String();
  // message.data = "Button clicked!";
  // publisher_->publish(message);

  stacked_layout_->setCurrentWidget(teleop_widget_);
}

}  // namespace drone_viz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(drone_viz::MainPanel, rviz_common::Panel)