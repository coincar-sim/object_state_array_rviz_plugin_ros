#include "object_prediction_visual.hpp"

#include <rviz/msg_conversions.h>
#include <util_rviz/util_rviz.hpp>

namespace object_state_array_rviz_plugin_ros {
ObjectPredictionVisual::ObjectPredictionVisual(Ogre::SceneManager* sm,
                                               Ogre::SceneNode* sn,
                                               const Msg& obj,
                                               const Parameters& p)
        : scene_manager_{sm}, scene_node_{sn}, obj_{obj}, params_(p), status_{obj.object_id} {

    if (params_.prediction_enabled) {
        makePrediction(scene_node_);
    }
}

void ObjectPredictionVisual::generateCorridor(const Ogre::Vector3& prevOgrePoint,
                                              const Ogre::Vector3& ogrePoint,
                                              std::vector<rviz::PointCloud::Point>& points,
                                              float min_y,
                                              float max_y) {

    // Add Billboard Border Points
    Ogre::Vector3 diff;
    diff.x = prevOgrePoint.x - ogrePoint.x;
    diff.y = prevOgrePoint.y - ogrePoint.y;

    Ogre::Vector3 hullExtensionLeft;
    hullExtensionLeft.x = diff.y;
    hullExtensionLeft.y = -diff.x;
    hullExtensionLeft.normalise();
    hullExtensionLeft *= std::abs(max_y); // left is positive y-values
    hullExtensionLeft += ogrePoint;

    Ogre::Vector3 hullExtensionRight;
    hullExtensionRight.x = -diff.y;
    hullExtensionRight.y = diff.x;
    hullExtensionRight.normalise();
    hullExtensionRight *= std::abs(min_y); // right is negative y-values
    hullExtensionRight += ogrePoint;

    prediction_line_left_.back()->addPoint(hullExtensionLeft);
    prediction_line_right_.back()->addPoint(hullExtensionRight);

    // Add pointcloud point
    rviz::PointCloud::Point p_out_left;
    p_out_left.position = hullExtensionLeft;
    p_out_left.color = params_.color;
    points.emplace_back(p_out_left);

    rviz::PointCloud::Point p_out_right;
    p_out_right.position = hullExtensionRight;
    p_out_right.color = params_.color;
    points.emplace_back(p_out_right);
}

void ObjectPredictionVisual::makePrediction(Ogre::SceneNode* sn) {

    if (obj_.motion_prediction.trajectories.empty()) {
        status_.level = Status::Level::Warn;
        status_.message += "no prediction available, ";
        return;
    }

    if (!params_.corridor && !params_.centerline) {
        return;
    }

    // Calc hull width
    // hull is in object frame
    float max_y = std::numeric_limits<float>::min(), min_y = std::numeric_limits<float>::max();
    std::for_each(obj_.hull.vertices.begin(), obj_.hull.vertices.end(), [&](const auto& p_in) {
        // calculate left outer point
        if (max_y < p_in.y) {
            max_y = p_in.y;
        }

        // calculate right outer point
        if (min_y > p_in.y) {
            min_y = p_in.y;
        }
    });

    prediction_center_.clear();
    prediction_line_left_.clear();
    prediction_line_right_.clear();

    // Prep Pointcloud
    prediction_point_cloud_ = std::make_shared<rviz::PointCloud>();
    prediction_point_cloud_->setRenderMode(rviz::PointCloud::RenderMode::RM_SPHERES);
    prediction_point_cloud_->setDimensions(params_.size, params_.size, params_.size);
    prediction_point_cloud_->setAlpha(params_.color.a);

    for (ulong i = 0; i < obj_.motion_prediction.trajectories.size() && i < params_.max_num_trajectories; i++) {
        const auto& prediction{obj_.motion_prediction.trajectories[i]};
        if (prediction.motion_states.empty()) {
            continue;
        }

        // Prep BillboardLine
        prediction_line_left_.push_back(std::make_shared<rviz::BillboardLine>(scene_manager_, sn));
        prediction_line_left_.back()->setLineWidth(params_.width);
        prediction_line_left_.back()->setColor(params_.color.r, params_.color.g, params_.color.b, params_.color.a);

        prediction_line_right_.push_back(std::make_shared<rviz::BillboardLine>(scene_manager_, sn));
        prediction_line_right_.back()->setLineWidth(params_.width);
        prediction_line_right_.back()->setColor(params_.color.r, params_.color.g, params_.color.b, params_.color.a);

        prediction_center_.push_back(std::make_shared<rviz::BillboardLine>(scene_manager_, sn));
        prediction_center_.back()->setLineWidth(params_.width);
        prediction_center_.back()->setColor(params_.color.r, params_.color.g, params_.color.b, params_.color.a);

        const ulong num_lines = (params_.centerline ? 1 : 0) + (params_.corridor ? 2 : 0);


        std::vector<rviz::PointCloud::Point> points(prediction.motion_states.size() * num_lines);
        for (size_t c = 0; c < prediction.motion_states.size(); c++) {
            const automated_driving_msgs::MotionState& point{prediction.motion_states[c]};
            if (params_.time_horizon > 0 &&
                point.header.stamp > obj_.header.stamp + ros::Duration(params_.time_horizon)) {
                break;
            }

            const Ogre::Vector3 ogrePoint = rviz::pointMsgToOgre(point.pose.pose.position);
            const Ogre::Vector3 prevOgrePoint =
                rviz::pointMsgToOgre(prediction.motion_states[c - 1].pose.pose.position);

            if (c > 0 && params_.corridor) {
                generateCorridor(prevOgrePoint, ogrePoint, points, min_y, max_y);
            }

            if (params_.centerline) {
                // Add billboard point
                prediction_center_.back()->addPoint(ogrePoint);

                // Add pointcloud point
                rviz::PointCloud::Point p_out_center;
                p_out_center.position = ogrePoint;
                p_out_center.color = params_.color;
                points.emplace_back(p_out_center);
            }
        }
        prediction_point_cloud_->addPoints(&(points.at(0)), points.size());
    }
    sn->attachObject(prediction_point_cloud_.get());
}

} // namespace object_state_array_rviz_plugin_ros
