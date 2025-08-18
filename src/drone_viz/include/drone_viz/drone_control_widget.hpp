#ifndef DRONE_CONTROL_WIDGET_HPP
#define DRONE_CONTROL_WIDGET_HPP

#include <QWidget>

class QStackedLayout;
namespace drone_viz { class MainPanel; }

class DroneControlWidget : public QWidget
{
    Q_OBJECT
public:
    explicit DroneControlWidget(drone_viz::MainPanel* parent_panel);
private:
    QStackedLayout* stacked_layout_;
    QWidget* main_widget_;
    QWidget* teleop_widget_;
    QWidget* autonomous_widget_;
};

#endif // DRONE_CONTROL_WIDGET_HPP