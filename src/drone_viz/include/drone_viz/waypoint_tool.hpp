#ifndef DRONE_VIZ__WAYPOINT_TOOL_HPP_
#define DRONE_VIZ__WAYPOINT_TOOL_HPP_

#include <rviz_common/tool.hpp>

namespace drone_viz
{

class WaypointTool : public rviz_common::Tool
{
  Q_OBJECT
public:
  WaypointTool();
  ~WaypointTool() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
};

}  // namespace drone_viz

#endif  // DRONE_VIZ__WAYPOINT_TOOL_HPP_
