#pragma once

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <polygonal_map_msgs/msg/line_map.hpp>

namespace polygonal_map_rviz_plugin {

class LineMapDisplay : public rviz_common::MessageFilterDisplay<polygonal_map_msgs::msg::LineMap>
{
Q_OBJECT
public:
  LineMapDisplay();

  void reset() override;

Q_SIGNALS:
  void mapReceived();

protected Q_SLOTS:
  void updateVisual();

protected:
  void processMessage(polygonal_map_msgs::msg::LineMap::ConstSharedPtr msg) override;
  void clearVisual();

  rviz_common::properties::BoolProperty *draw_free_space_property_;
  rviz_common::properties::ColorProperty *free_space_color_property_;
  rviz_common::properties::FloatProperty *free_space_alpha_property_;
  rviz_common::properties::FloatProperty *free_space_dilation_;
  rviz_common::properties::FloatProperty *line_height_property_;
  rviz_common::properties::ColorProperty *line_color_property_;
  rviz_common::properties::FloatProperty *line_alpha_property_;

  polygonal_map_msgs::msg::LineMap current_map_;
  rclcpp::Subscription<polygonal_map_msgs::msg::LineMap>::SharedPtr map_sub_;
  bool loaded_;

  inline static Ogre::MaterialPtr free_space_material_ = nullptr;
  inline static Ogre::MaterialPtr line_material_ = nullptr;
};

} // namespace polygonal_map_rviz_plugin
