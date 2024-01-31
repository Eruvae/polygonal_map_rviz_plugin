#include "polygonal_map_rviz_plugin/line_map_display.hpp"

#include <random>
#include <Ogre.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace polygonal_map_rviz_plugin
{

LineMapDisplay::LineMapDisplay() : rviz_common::MessageFilterDisplay<polygonal_map_msgs::msg::LineMap>(), loaded_(false)
{
    draw_free_space_property_ = new rviz_common::properties::BoolProperty("Draw free space", true, "Draw free space in between obstacles", this);
    free_space_color_property_ = new rviz_common::properties::ColorProperty("Free space color", QColor(255, 255, 255), "Color of free space", this);
    line_height_property_ = new rviz_common::properties::FloatProperty("Line height", 0.3, "Height of lines", this);
    line_color_property_ = new rviz_common::properties::ColorProperty("Line color", QColor(0, 0, 0), "Color of lines", this);

    connect(this, SIGNAL(mapReceived()), this, SLOT(updateVisual()));
    connect(draw_free_space_property_, SIGNAL(changed()), this, SLOT(updateVisual()));
    connect(free_space_color_property_, SIGNAL(changed()), this, SLOT(updateVisual()));
    connect(line_height_property_, SIGNAL(changed()), this, SLOT(updateVisual()));
    connect(line_color_property_, SIGNAL(changed()), this, SLOT(updateVisual()));
}

void LineMapDisplay::reset()
{
    rviz_common::MessageFilterDisplay<polygonal_map_msgs::msg::LineMap>::reset();
    clearVisual();
}

void LineMapDisplay::clearVisual()
{
    scene_node_->detachAllObjects();
    scene_node_->removeAndDestroyAllChildren();
}

void LineMapDisplay::updateVisual()
{
    clearVisual();

    if (!loaded_)
    {
        return;
    }

    geometry_msgs::msg::Point min, max;
    min.x = min.y = std::numeric_limits<double>::max();
    max.x = max.y = std::numeric_limits<double>::lowest();

    for (const auto &line : current_map_.lines)
    {
        if (!std::isfinite(line.start.x) || !std::isfinite(line.start.y) || !std::isfinite(line.end.x) || !std::isfinite(line.end.y))
        {
            continue; // invalid line
        }

        Ogre::ManualObject *mo = scene_manager_->createManualObject();
        Ogre::ColourValue line_col = line_color_property_->getOgreColor();
        float height = line_height_property_->getFloat();

        mo->estimateVertexCount(4);
        mo->estimateIndexCount(12);
        mo->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        bool shape_valid = true;

        mo->position(line.start.x, line.start.y, 0); mo->colour(line_col);
        mo->position(line.end.x, line.end.y, 0); mo->colour(line_col);
        mo->position(line.end.x, line.end.y, height); mo->colour(line_col);
        mo->position(line.start.x, line.start.y, height); mo->colour(line_col);

        mo->quad(0, 1, 2, 3);
        mo->quad(3, 2, 1, 0);
        mo->end();
        
        min.x = std::min(min.x, line.start.x); min.y = std::min(min.y, line.start.y);
        max.x = std::max(max.x, line.start.x); max.y = std::max(max.y, line.start.y);
        min.x = std::min(min.x, line.end.x); min.y = std::min(min.y, line.end.y);
        max.x = std::max(max.x, line.end.x); max.y = std::max(max.y, line.end.y);
        
        scene_node_->attachObject(mo);
    }
    if (draw_free_space_property_->getBool())
    {
        Ogre::ManualObject *mo = scene_manager_->createManualObject();
        mo->estimateVertexCount(4);
        mo->estimateIndexCount(6);
        mo->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        Ogre::ColourValue free_col = free_space_color_property_->getOgreColor();
        mo->position(min.x, min.y, 0); mo->colour(free_col);
        mo->position(max.x, min.y, 0); mo->colour(free_col);
        mo->position(max.x, max.y, 0); mo->colour(free_col);
        mo->position(min.x, max.y, 0); mo->colour(free_col);
        mo->quad(0, 1, 2, 3);
        mo->end();
        scene_node_->attachObject(mo);
    }
}

void LineMapDisplay::processMessage(polygonal_map_msgs::msg::LineMap::ConstSharedPtr msg)
{
    current_map_ = *msg;
    loaded_ = true;
    //setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Map received");
    rclcpp::Time msg_time(msg->header.stamp, RCL_ROS_TIME);
    if (!updateFrame(msg->header.frame_id, msg_time)) {
        setMissingTransformToFixedFrame(msg->header.frame_id);
        return;
    }
    setTransformOk();
    Q_EMIT mapReceived();
}

} // namespace polygonal_map_rviz_plugin

PLUGINLIB_EXPORT_CLASS(polygonal_map_rviz_plugin::LineMapDisplay, rviz_common::Display)