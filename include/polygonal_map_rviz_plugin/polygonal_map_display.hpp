#pragma once

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <polygonal_map_msgs/msg/polygonal_map.hpp>

namespace polygonal_map_rviz_plugin {

class PolygonalMapDisplay : public rviz_common::MessageFilterDisplay<polygonal_map_msgs::msg::PolygonalMap>
{
Q_OBJECT
public:
  PolygonalMapDisplay();

  void reset() override;

Q_SIGNALS:
  void mapReceived();

protected Q_SLOTS:
  void updateVisual();

protected:
  void processMessage(polygonal_map_msgs::msg::PolygonalMap::ConstSharedPtr msg) override;
  void clearVisual();

  rviz_common::properties::BoolProperty *draw_free_space_property_;
  rviz_common::properties::ColorProperty *free_space_color_property_;
  rviz_common::properties::FloatProperty *obstacle_height_property_;
  rviz_common::properties::ColorProperty *obstacle_color_property_;

  polygonal_map_msgs::msg::PolygonalMap current_map_;
  rclcpp::Subscription<polygonal_map_msgs::msg::PolygonalMap>::SharedPtr map_sub_;
  bool loaded_;
};

} // namespace polygonal_map_rviz_plugin
