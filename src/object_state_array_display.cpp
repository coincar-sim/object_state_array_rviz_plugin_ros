/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "object_state_array_display.hpp"

namespace object_state_array_rviz_plugin_ros {

ObjectStateArrayDisplay::ObjectStateArrayDisplay() {

    /**
     * Mesh or Primitive
     */
    prop_visualization_ =
        std::make_unique<rviz::Property>("Visualization", QVariant(), "Object display options.", this);
    prop_dropdown_visu_ = std::make_unique<rviz::EnumProperty>("Method",
                                                               "Meshes",
                                                               "Whether to show mesh, primitives or raw points",
                                                               prop_visualization_.get(),
                                                               SLOT(update()),
                                                               this);
    prop_dropdown_visu_->addOption("Primitives", 0);
    prop_dropdown_visu_->addOption("Meshes", 1);
    prop_dropdown_visu_->addOption("Point Cloud", 2);

    /**
     * Coloring
     */
    prop_coloring_by_class_ = std::make_unique<rviz::BoolProperty>("Coloring by class",
                                                                   true,
                                                                   "Whether to use class-dependent coloring.",
                                                                   prop_visualization_.get(),
                                                                   SLOT(updateColoring()),
                                                                   this);

    prop_coloring_alpha_ = std::make_unique<rviz::FloatProperty>(
        "Alpha", 1.0, "0 is transparent, 1 is opaque.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_alpha_->setMin(0.0);
    prop_coloring_alpha_->setMax(1.0);
    prop_coloring_unknown_ = std::make_unique<rviz::ColorProperty>("Unknown",
                                                                   QColor(100, 100, 100),
                                                                   "Color to draw all objects of unknown class.",
                                                                   prop_visualization_.get(),
                                                                   SLOT(update()),
                                                                   this);
    prop_coloring_vehicle_ = std::make_unique<rviz::ColorProperty>(
        "Vehicles", QColor(255, 0, 0), "Vehicle coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_pedestrian_ = std::make_unique<rviz::ColorProperty>(
        "Pedestrians", QColor(0, 255, 0), "Pedestrian coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_bike_ = std::make_unique<rviz::ColorProperty>(
        "Bikes", QColor(0, 0, 255), "Bike coloring.", prop_visualization_.get(), SLOT(update()), this);

    /**
     * Arrows
     */
    prop_show_arrows_ = std::make_unique<rviz::BoolProperty>(
        "Arrows", true, "Whether to show velocity arrows or not.", this, SLOT(updateArrows()));
    prop_arrow_v_min_ =
        std::make_unique<rviz::FloatProperty>("Minimum velocity",
                                              0,
                                              "The minimum velocity in m/s at which a velocity arrow is still shown",
                                              prop_show_arrows_.get(),
                                              SLOT(update()),
                                              this);
    prop_arrow_v_max_ = std::make_unique<rviz::FloatProperty>(
        "Maximum velocity",
        30,
        "The velocity in m/s at which the arrow has maximum length and is clipped afterwards.",
        prop_show_arrows_.get(),
        SLOT(update()),
        this);
    prop_arrow_length_ = std::make_unique<rviz::FloatProperty>(
        "Length", 10, "The maximum arrow length in meters.", prop_show_arrows_.get(), SLOT(update()), this);

    /**
     * Text
     */
    prop_text_show_ = std::make_unique<rviz::BoolProperty>(
        "Text", true, "Whether to show textual information for objects.", this, SLOT(update()));
    prop_text_size_ = std::make_unique<rviz::FloatProperty>(
        "Size", 0.4, "The text size.", prop_text_show_.get(), SLOT(update()), this);
    prop_text_debug_ = std::make_unique<rviz::BoolProperty>(
        "Debug info", false, "Whether to show debug info for objects.", prop_text_show_.get(), SLOT(update()), this);

    /**
     * Point cloud
     */
    prop_point_cloud_ = std::make_unique<rviz::Property>(
        "Point Cloud", QVariant(), "Point cloud visualization options.", prop_visualization_.get());
    prop_point_cloud_point_size_ = std::make_unique<rviz::FloatProperty>(
        "Point size", 0.05, "Point size", prop_point_cloud_.get(), SLOT(update()), this);

    /**
     * Prediction
     */
    prop_prediction_ = std::make_unique<rviz::BoolProperty>(
        "Prediction", false, "Whether to display prediction or not.", this, SLOT(update()));
    prop_prediction_centerline_ = std::make_unique<rviz::BoolProperty>(
        "Centerline", true, "Whether to display the centerline.", prop_prediction_.get(), SLOT(update()), this);
    prop_prediction_corridor_ = std::make_unique<rviz::BoolProperty>(
        "Corridor", false, "Whether to display the corridor.", prop_prediction_.get(), SLOT(update()), this);
    prop_prediction_point_cloud_point_size_ = std::make_unique<rviz::FloatProperty>(
        "Point Size", 0.5, "Point size", prop_prediction_.get(), SLOT(update()), this);
    prop_prediction_billboard_width_ = std::make_unique<rviz::FloatProperty>(
        "Line Width", 0.05, "Line width", prop_prediction_.get(), SLOT(update()), this);
    prop_prediction_color_ = std::make_unique<rviz::ColorProperty>(
        "Color", QColor(0, 0, 0), "Prediction coloring.", prop_prediction_.get(), SLOT(update()), this);
    prop_prediction_horizon_ =
        std::make_unique<rviz::FloatProperty>("Time Horizon",
                                              3.0,
                                              "Prediction Horizon to display in sec (<= 0 enables all)",
                                              prop_prediction_.get(),
                                              SLOT(update()),
                                              this);
    prop_prediction_number_traj_ =
        std::make_unique<rviz::IntProperty>("Number of Trajectories",
                                            2,
                                            "Maximum number of predicted trajectories to display",
                                            prop_prediction_.get(),
                                            SLOT(update()),
                                            this);
}

void ObjectStateArrayDisplay::updateDropdown() {
    params_visual_.visualization_mode =
        static_cast<ObjectStateVisual::Parameters::VisualizationMode>(prop_dropdown_visu_->getOptionInt());

    prop_point_cloud_->setHidden(prop_dropdown_visu_->getOptionInt() != 2);
}

void ObjectStateArrayDisplay::updateColoring() {
    prop_coloring_vehicle_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_pedestrian_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_bike_->setHidden(!prop_coloring_by_class_->getBool());
    processMessage(msg_last_);
}

void ObjectStateArrayDisplay::updateArrows() {
    prop_arrow_v_min_->setHidden(!prop_show_arrows_->getBool());
    prop_arrow_v_max_->setHidden(!prop_show_arrows_->getBool());
    prop_arrow_length_->setHidden(!prop_show_arrows_->getBool());
    processMessage(msg_last_);
}

void ObjectStateArrayDisplay::updateParameters() {
    params_visual_.make_arrow = prop_show_arrows_->getBool();
    params_visual_.make_text = prop_text_show_->getBool();

    params_visual_.arrow_length = prop_arrow_length_->getFloat();
    params_visual_.arrow_v_max = prop_arrow_v_max_->getFloat();
    params_visual_.arrow_v_min = prop_arrow_v_min_->getFloat();

    params_visual_.text_debug = prop_text_debug_->getBool();
    params_visual_.text_font_size = prop_text_size_->getFloat();
    params_visual_.text_v_min = params_visual_.arrow_v_min;

    params_visual_.point_cloud_point_size = prop_point_cloud_point_size_->getFloat();

    params_visual_prediction_.prediction_enabled = prop_prediction_->getBool();
    params_visual_prediction_.size = prop_prediction_point_cloud_point_size_->getFloat();
    params_visual_prediction_.width = prop_prediction_billboard_width_->getFloat();
    params_visual_prediction_.color = prop_prediction_color_->getOgreColor();
    params_visual_prediction_.time_horizon = static_cast<double>(prop_prediction_horizon_->getFloat());

    params_visual_prediction_.centerline = prop_prediction_centerline_->getBool();
    params_visual_prediction_.corridor = prop_prediction_corridor_->getBool();
    if (!params_visual_prediction_.centerline && !params_visual_prediction_.corridor) {
        params_visual_prediction_.prediction_enabled = false;
    }

    params_visual_prediction_.max_num_trajectories = prop_prediction_number_traj_->getInt();

    updateDropdown();
}

Ogre::ColourValue ObjectStateArrayDisplay::colorFromClassification(
    const automated_driving_msgs::ObjectClassification& classification) {

    Ogre::ColourValue color;

    if (!prop_coloring_by_class_->getBool() || classification.classes_with_probabilities.empty()) {
        color = prop_coloring_unknown_->getOgreColor();
    } else {
        switch (classification.classes_with_probabilities.front().classification) {

        case automated_driving_msgs::ObjectClassification::CAR:
            color = prop_coloring_vehicle_->getOgreColor();
            break;

        case automated_driving_msgs::ObjectClassification::PEDESTRIAN:
            color = prop_coloring_pedestrian_->getOgreColor();
            break;

        case automated_driving_msgs::ObjectClassification::BICYCLE:
            color = prop_coloring_bike_->getOgreColor();
            break;

        default:
            color = prop_coloring_unknown_->getOgreColor();
        }
    }
    color.a = prop_coloring_alpha_->getFloat();
    return color;
}

void ObjectStateArrayDisplay::processMessage(const Msg::ConstPtr& msg) {
    try {
        msg_last_ = msg;
        bool transforms_ok{true};
        bool prediction_ok{true};
        std::pair<size_t, size_t> status_hulls{std::make_pair(0, 0)};

        updateParameters();
        visuals_.clear();
        visuals_prediction_.clear();

        if (msg == nullptr || messages_received_ == 0) {
            setStatusStd(rviz::StatusProperty::Level::Warn, "Topic", "No message received");
            return;
        }

        if ((msg->header.stamp - ros::Time::now()).toSec() > 1.0) {
            setStatusStd(rviz::StatusProperty::Level::Warn, "Topic", "Message delay > 1 second");
        }

        visuals_.reserve(msg->objects.size());
        visuals_prediction_.reserve(msg->objects.size());
        for (const auto& obj : msg->objects) {
            params_visual_.color = colorFromClassification(obj.classification);
            /**
             * Get transform for state
             */
            Ogre::Quaternion orientation;
            Ogre::Vector3 position;
            if (!context_->getFrameManager()->transform(
                    obj.motion_state.header, obj.motion_state.pose.pose, position, orientation)) {
                setStatusStd(rviz::StatusProperty::Level::Error,
                             "Frame",
                             "No transformation found for frame '" + obj.motion_state.header.frame_id + "'");
                transforms_ok = false;
            }
            if (position.isNaN() || orientation.isNaN()) {
                setStatusStd(rviz::StatusProperty::Level::Error, "Frame", "Motion state contains NANs");
                transforms_ok = false;
                continue;
            }

            if (obj.hull.triangles.empty()) {
                status_hulls.first++;
            }
            if (obj.hull.vertices.empty()) {
                status_hulls.second++;
                continue;
            }
            visuals_.emplace_back(context_->getSceneManager(),
                                  scene_node_->createChildSceneNode(position, orientation),
                                  obj,
                                  params_visual_);

            /**
             * Get transform for prediction
             */
            if (obj.motion_prediction.header.frame_id == "" or obj.motion_prediction.trajectories.empty()) {
                setStatusStd(params_visual_prediction_.prediction_enabled ? rviz::StatusProperty::Level::Error
                                                                          : rviz::StatusProperty::Level::Warn,
                             "Prediction",
                             "No prediction found");
                prediction_ok = false;
                continue;
            }
            if (params_visual_prediction_.prediction_enabled) {
                if (!context_->getFrameManager()->transform(
                        obj.motion_prediction.header, geometry_msgs::Pose(), position, orientation)) {
                    setStatusStd(rviz::StatusProperty::Level::Error,
                                 "Frame",
                                 "No transformation found for frame '" + obj.motion_prediction.header.frame_id + "'");
                    transforms_ok = false;
                }
                if (position.isNaN() || orientation.isNaN()) {
                    setStatusStd(rviz::StatusProperty::Level::Error, "Frame", "Motion prediction contains NANs");
                    transforms_ok = false;
                    continue;
                }
                visuals_prediction_.emplace_back(context_->getSceneManager(),
                                                 scene_node_->createChildSceneNode(position, orientation),
                                                 obj,
                                                 params_visual_prediction_);
            }
        }

        if (transforms_ok) {
            setStatusStd(rviz::StatusProperty::Level::Ok, "Frame", "'" + msg->header.frame_id + "'");
        }

        if (prediction_ok) {
            setStatusStd(rviz::StatusProperty::Level::Ok, "Prediction", "ok");
        }

        if (status_hulls.first) {
            setStatusStd(rviz::StatusProperty::Level::Warn,
                         "Hull Meshes",
                         "No hull meshes available for " + std::to_string(status_hulls.first) +
                             " objects. Mesh visualization will not be available for these objects.");
        } else {
            setStatusStd(rviz::StatusProperty::Level::Ok, "Hull Meshes", "ok");
        }

        if (status_hulls.second) {
            setStatusStd(rviz::StatusProperty::Level::Warn,
                         "Hull Points",
                         "No hull points available for " + std::to_string(status_hulls.second) + " objects.");
        } else {
            setStatusStd(rviz::StatusProperty::Level::Ok, "Hull Points", "ok");
        }

        rviz::StatusProperty::Level visuals_level{rviz::StatusProperty::Level::Ok};
        std::string visuals_message;
        std::for_each(std::begin(visuals_), std::end(visuals_), [&](const auto& visual) {
            const auto& status{visual.getStatus()};
            if (status.level != rviz::StatusProperty::Level::Ok) {
                visuals_level = rviz::StatusProperty::Level::Warn;
                visuals_message += status.message;
            }
        });

        std::for_each(std::begin(visuals_prediction_), std::end(visuals_prediction_), [&](const auto& visual) {
            const auto& status{visual.getStatus()};
            if (status.level != rviz::StatusProperty::Level::Ok) {
                visuals_level = rviz::StatusProperty::Level::Warn;
                visuals_message += status.message;
            }
        });

        setStatusStd(visuals_level, "Visuals", visuals_message);
    } catch (std::exception& e) {
        ROS_ERROR_STREAM(e.what());
        ROS_ERROR_STREAM("Failed!");
    }
}

} // namespace object_state_array_rviz_plugin_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(object_state_array_rviz_plugin_ros::ObjectStateArrayDisplay, rviz::Display)
