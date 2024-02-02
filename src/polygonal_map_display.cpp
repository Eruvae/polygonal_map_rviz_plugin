#include "polygonal_map_rviz_plugin/polygonal_map_display.hpp"

#include <random>
#include <Ogre.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include "polygonal_map_rviz_plugin/earcut.hpp"

namespace mapbox {
namespace util {

template <>
struct nth<0, geometry_msgs::msg::Point32> {
    inline static geometry_msgs::msg::Point32::_x_type get(const geometry_msgs::msg::Point32 &t) {
        return t.x;
    }
};

template <>
struct nth<1, geometry_msgs::msg::Point32> {
    inline static geometry_msgs::msg::Point32::_y_type get(const geometry_msgs::msg::Point32 &t) {
        return t.y;
    }
};

} // namespace util
} // namespace mapbox

namespace polygonal_map_rviz_plugin
{

PolygonalMapDisplay::PolygonalMapDisplay() : rviz_common::MessageFilterDisplay<polygonal_map_msgs::msg::PolygonalMap>(), loaded_(false)
{
    auto [resource, newly_created] = Ogre::MaterialManager::getSingleton().createOrRetrieve("polygonal_map_free_space_material", "rviz_rendering");
    free_space_material_ = Ogre::static_pointer_cast<Ogre::Material>(resource);
    if (newly_created)
    {
        free_space_material_->setReceiveShadows(false);
        free_space_material_->getTechnique(0)->setLightingEnabled(false);
        free_space_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        free_space_material_->setDepthWriteEnabled(false);
    }
    std::tie(resource, newly_created) = Ogre::MaterialManager::getSingleton().createOrRetrieve("polygonal_map_obstacle_material", "rviz_rendering");
    obstacle_material_ = Ogre::static_pointer_cast<Ogre::Material>(resource);
    if (newly_created)
    {
        obstacle_material_->setReceiveShadows(false);
        obstacle_material_->getTechnique(0)->setLightingEnabled(false);
        obstacle_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        obstacle_material_->setDepthWriteEnabled(false);
    }

    draw_free_space_property_ = new rviz_common::properties::BoolProperty("Draw free space", true, "Draw free space in between obstacles", this, SLOT(updateVisual()));
    free_space_color_property_ = new rviz_common::properties::ColorProperty("Free space color", QColor(255, 255, 255), "Color of free space", this, SLOT(updateVisual()));
    free_space_alpha_property_ = new rviz_common::properties::FloatProperty("Free space alpha", 0.7f, "Transparency of free space", this, SLOT(updateVisual()));
    free_space_alpha_property_->setMin(0); free_space_alpha_property_->setMax(1);
    free_space_dilation_ = new rviz_common::properties::FloatProperty("Free space dilation", 0.2f, "Free space dilation beyond obstacle range", this, SLOT(updateVisual()));
    free_space_dilation_->setMin(0);
    obstacle_height_property_ = new rviz_common::properties::FloatProperty("Height of obstacles", 0.0f, "Height of obstacles", this, SLOT(updateVisual()));
    obstacle_color_property_ = new rviz_common::properties::ColorProperty("Obstacle color", QColor(0, 0, 0), "Color of obstacles", this, SLOT(updateVisual()));
    obstacle_alpha_property_ = new rviz_common::properties::FloatProperty("Obstacle alpha", 1.0f, "Transparency of obstacles", this, SLOT(updateVisual()));
    obstacle_alpha_property_->setMin(0); obstacle_alpha_property_->setMax(1);

    connect(this, SIGNAL(mapReceived()), this, SLOT(updateVisual()));
}

void PolygonalMapDisplay::reset()
{
    rviz_common::MessageFilterDisplay<polygonal_map_msgs::msg::PolygonalMap>::reset();
    clearVisual();
}

void PolygonalMapDisplay::clearVisual()
{
    scene_node_->detachAllObjects();
    scene_node_->removeAndDestroyAllChildren();
}

void PolygonalMapDisplay::updateVisual()
{
    clearVisual();

    if (!loaded_)
    {
        return;
    }

    geometry_msgs::msg::Point32 min, max;
    min.x = min.y = std::numeric_limits<float>::max();
    max.x = max.y = std::numeric_limits<float>::lowest();

    if (current_map_.obstacles.empty())
    {
        min.x = min.y = max.x = max.y = 0;
    }

    for (const auto &obs : current_map_.obstacles)
    {
        if (obs.points.size() < 2) // single or no point
            continue;

        Ogre::ColourValue obs_col = obstacle_color_property_->getOgreColor();
        obs_col.a = obstacle_alpha_property_->getFloat();
        float height = obstacle_height_property_->getFloat();

        if (obs.points.size() == 2) // is a line
        {
            if (!std::isfinite(obs.points[0].x) || !std::isfinite(obs.points[0].y) || !std::isfinite(obs.points[1].x) || !std::isfinite(obs.points[1].y))
            {
                continue; // invalid line
            }
            Ogre::ManualObject *mo = scene_manager_->createManualObject();
            if (height == 0)
            {
                mo->estimateVertexCount(2);
                mo->begin(obstacle_material_, Ogre::RenderOperation::OT_LINE_LIST);
                mo->position(obs.points[0].x, obs.points[0].y, 0); mo->colour(obs_col);
                mo->position(obs.points[1].x, obs.points[1].y, 0); mo->colour(obs_col);
                mo->end();
                
            }
            else
            {
                mo->estimateVertexCount(4);
                mo->estimateIndexCount(12);
                mo->begin(obstacle_material_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

                mo->position(obs.points[0].x, obs.points[0].y, 0); mo->colour(obs_col);
                mo->position(obs.points[1].x, obs.points[1].y, 0); mo->colour(obs_col);
                mo->position(obs.points[1].x, obs.points[1].y, height); mo->colour(obs_col);
                mo->position(obs.points[0].x, obs.points[0].y, height); mo->colour(obs_col);

                mo->quad(0, 1, 2, 3);
                mo->quad(3, 2, 1, 0);
                mo->end();
            }
            
            min.x = std::min(min.x, obs.points[0].x); min.y = std::min(min.y, obs.points[0].y);
            max.x = std::max(max.x, obs.points[0].x); max.y = std::max(max.y, obs.points[0].y);
            min.x = std::min(min.x, obs.points[1].x); min.y = std::min(min.y, obs.points[1].y);
            max.x = std::max(max.x, obs.points[1].x); max.y = std::max(max.y, obs.points[1].y);

            scene_node_->attachObject(mo);
            continue;
        }

        // draw actual polygon, use earcut to triangulate
        std::vector<std::vector<geometry_msgs::msg::Point32>> pg;
        pg.push_back(obs.points);
        std::vector<uint32_t> indices = mapbox::earcut(pg);
        uint32_t num_points = static_cast<uint32_t>(obs.points.size());

        Ogre::ManualObject *mo = scene_manager_->createManualObject();
        if (height == 0) // draw 2D polygons
        {
            mo->estimateVertexCount(num_points);
            mo->estimateIndexCount(2 * indices.size());
        }
        else
        {
            mo->estimateVertexCount(2 * num_points);
            mo->estimateIndexCount(2 * indices.size() + 6 * num_points);
        }
        mo->begin(obstacle_material_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
        bool shape_valid = true;
        for (const auto &point : obs.points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y))
            {
                shape_valid = false;
                break;
            }
            min.x = std::min(min.x, point.x); min.y = std::min(min.y, point.y);
            max.x = std::max(max.x, point.x); max.y = std::max(max.y, point.y);
            mo->position(point.x, point.y, height); // add vertices on specified height
            mo->colour(obs_col);
        }
        if (!shape_valid)
        {
            mo->end();
            delete mo;
            continue;
        }
        if (height == 0)
        {
            for (size_t i=0; i < indices.size() - 2; i+=3)
            {
                uint32_t i0 = indices[i];
                uint32_t i1 = indices[i+1];
                uint32_t i2 = indices[i+2];
                mo->triangle(i0, i1, i2);  // top triangles
                mo->triangle(i2, i1, i0);  // bottom triangles
            }
        }
        else
        {
            for (const auto &point : obs.points)
            {
                mo->position(point.x, point.y, 0); // add vertices on floor
                mo->colour(obs_col);
            }
            for (size_t i=0; i < indices.size() - 2; i+=3)
            {
                uint32_t i0 = indices[i];
                uint32_t i1 = indices[i+1];
                uint32_t i2 = indices[i+2];
                mo->triangle(i0, i1, i2);  // top triangles
                mo->triangle(i2 + num_points, i1 + num_points, i0 + num_points); // bottom triangles
                
            }
            for (uint32_t i=0; i < num_points - 1; i++)
            {
                uint32_t j = i + num_points;
                mo->quad(j, j+1, i+1, i); // sides
            }
            uint32_t li = num_points - 1;
            uint32_t lj = li + num_points;
            mo->quad(lj, num_points, 0, li); // last side
        }
        mo->end();
        scene_node_->attachObject(mo);
    }
    if (draw_free_space_property_->getBool())
    {
        float dilation = free_space_dilation_->getFloat();
        min.x -= dilation; min.y -= dilation;
        max.x += dilation; max.y += dilation;
        Ogre::ManualObject *mo = scene_manager_->createManualObject();
        mo->estimateVertexCount(4);
        mo->estimateIndexCount(6);
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

void PolygonalMapDisplay::processMessage(polygonal_map_msgs::msg::PolygonalMap::ConstSharedPtr msg)
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

PLUGINLIB_EXPORT_CLASS(polygonal_map_rviz_plugin::PolygonalMapDisplay, rviz_common::Display)