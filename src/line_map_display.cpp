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
    auto [resource, newly_created] = Ogre::MaterialManager::getSingleton().createOrRetrieve("line_map_free_space_material", "rviz_rendering");
    free_space_material_ = Ogre::static_pointer_cast<Ogre::Material>(resource);
    if (newly_created)
    {
        free_space_material_->setReceiveShadows(false);
        free_space_material_->getTechnique(0)->setLightingEnabled(false);
        free_space_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        free_space_material_->setDepthWriteEnabled(false);
    }
    std::tie(resource, newly_created) = Ogre::MaterialManager::getSingleton().createOrRetrieve("line_map_line_material", "rviz_rendering");
    line_material_ = Ogre::static_pointer_cast<Ogre::Material>(resource);
    if (newly_created)
    {
        line_material_->setReceiveShadows(false);
        line_material_->getTechnique(0)->setLightingEnabled(false);
        line_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        line_material_->setDepthWriteEnabled(false);
    }

    draw_free_space_property_ = new rviz_common::properties::BoolProperty("Draw free space", true, "Draw free space in between obstacles", this, SLOT(updateVisual()));
    free_space_color_property_ = new rviz_common::properties::ColorProperty("Free space color", QColor(255, 255, 255), "Color of free space", this, SLOT(updateVisual()));
    free_space_alpha_property_ = new rviz_common::properties::FloatProperty("Free space alpha", 0.7f, "Transparency of free space", this, SLOT(updateVisual()));
    free_space_alpha_property_->setMin(0); free_space_alpha_property_->setMax(1);
    free_space_dilation_ = new rviz_common::properties::FloatProperty("Free space dilation", 0.2f, "Free space dilation beyond line range", this, SLOT(updateVisual()));
    free_space_dilation_->setMin(0);
    line_height_property_ = new rviz_common::properties::FloatProperty("Line height", 0.0f, "Height of lines", this, SLOT(updateVisual()));
    line_color_property_ = new rviz_common::properties::ColorProperty("Line color", QColor(0, 0, 0), "Color of lines", this, SLOT(updateVisual()));
    line_alpha_property_ = new rviz_common::properties::FloatProperty("Line alpha", 1.0f, "Transparency of lines", this, SLOT(updateVisual()));
    line_alpha_property_->setMin(0); line_alpha_property_->setMax(1);

    connect(this, SIGNAL(mapReceived()), this, SLOT(updateVisual()));
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

    if (current_map_.lines.empty())
    {
        min.x = min.y = max.x = max.y = 0;
    }

    Ogre::ColourValue line_col = line_color_property_->getOgreColor();
    line_col.a = line_alpha_property_->getFloat();
    float height = line_height_property_->getFloat();

    for (const auto &line : current_map_.lines)
    {
        if (!std::isfinite(line.start.x) || !std::isfinite(line.start.y) || !std::isfinite(line.end.x) || !std::isfinite(line.end.y))
        {
            continue; // invalid line
        }

        Ogre::ManualObject *mo = scene_manager_->createManualObject();
        if (height == 0)
        {
            mo->estimateVertexCount(2);
            mo->begin(line_material_, Ogre::RenderOperation::OT_LINE_LIST);
            mo->position(line.start.x, line.start.y, 0); mo->colour(line_col);
            mo->position(line.end.x, line.end.y, 0); mo->colour(line_col);
            mo->end();
        }
        else
        {
            mo->estimateVertexCount(4);
            mo->estimateIndexCount(12);
            mo->begin(line_material_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

            mo->position(line.start.x, line.start.y, 0); mo->colour(line_col);
            mo->position(line.end.x, line.end.y, 0); mo->colour(line_col);
            mo->position(line.end.x, line.end.y, height); mo->colour(line_col);
            mo->position(line.start.x, line.start.y, height); mo->colour(line_col);

            mo->quad(0, 1, 2, 3);
            mo->quad(3, 2, 1, 0);
            mo->end();
        }
        
        min.x = std::min(min.x, line.start.x); min.y = std::min(min.y, line.start.y);
        max.x = std::max(max.x, line.start.x); max.y = std::max(max.y, line.start.y);
        min.x = std::min(min.x, line.end.x); min.y = std::min(min.y, line.end.y);
        max.x = std::max(max.x, line.end.x); max.y = std::max(max.y, line.end.y);
        
        scene_node_->attachObject(mo);
    }
    if (draw_free_space_property_->getBool())
    {
        float dilation = free_space_dilation_->getFloat();
        min.x -= dilation; min.y -= dilation;
        max.x += dilation; max.y += dilation;
        Ogre::ManualObject *mo = scene_manager_->createManualObject();
        mo->estimateVertexCount(4);
        mo->estimateIndexCount(12);
        mo->begin(free_space_material_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
        Ogre::ColourValue free_col = free_space_color_property_->getOgreColor();
        free_col.a = free_space_alpha_property_->getFloat();
        mo->position(min.x, min.y, 0); mo->colour(free_col);
        mo->position(max.x, min.y, 0); mo->colour(free_col);
        mo->position(max.x, max.y, 0); mo->colour(free_col);
        mo->position(min.x, max.y, 0); mo->colour(free_col);
        mo->quad(0, 1, 2, 3);
        mo->quad(3, 2, 1, 0);
        mo->setRenderQueueGroupAndPriority(0, 0);
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