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
    prop_dropdown_visu_ = std::make_unique<rviz::EnumProperty>("Meshes/Primitives",
                                                               "Meshes",
                                                               "Whether to show the mesh or an object primitive.",
                                                               prop_visualization_.get(),
                                                               SLOT(update()),
                                                               this);
    prop_dropdown_visu_->addOption("Primitives", 1);
    prop_dropdown_visu_->addOption("Meshes", 2);


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
    prop_coloring_road_ = std::make_unique<rviz::ColorProperty>(
        "Road", QColor(128, 64, 128), "Road coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_sidewalk_ = std::make_unique<rviz::ColorProperty>(
        "Sidewalks", QColor(244, 35, 232), "Sidewalk coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_building_ = std::make_unique<rviz::ColorProperty>(
        "Buildings", QColor(70, 70, 70), "Building coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_wall_ = std::make_unique<rviz::ColorProperty>(
        "Wall", QColor(102, 102, 156), "Wall coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_fence_ = std::make_unique<rviz::ColorProperty>(
        "Fence", QColor(190, 153, 153), "Fence coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_pole_ = std::make_unique<rviz::ColorProperty>(
        "Pole", QColor(153, 153, 153), "Pole coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_traffic_light_ = std::make_unique<rviz::ColorProperty>(
        "TrafficLight", QColor(250, 170, 30), "TrafficLight coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_traffic_sign_ = std::make_unique<rviz::ColorProperty>(
        "TrafficSign", QColor(220, 220, 0), "TrafficSign coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_vegetation_ = std::make_unique<rviz::ColorProperty>(
        "Vegetation", QColor(107, 142, 35), "Vegetation coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_terrain_ = std::make_unique<rviz::ColorProperty>(
        "Terrain", QColor(152, 251, 152), "Terrain coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_sky_ = std::make_unique<rviz::ColorProperty>(
        "Sky", QColor(70, 130, 180), "Sky coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_person_ = std::make_unique<rviz::ColorProperty>(
        "Person", QColor(220, 20, 60), "Person coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_rider_ = std::make_unique<rviz::ColorProperty>(
        "Rider", QColor(255, 0, 0), "Rider coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_car_ = std::make_unique<rviz::ColorProperty>(
        "Car", QColor(0, 0, 142), "Car coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_truck_ = std::make_unique<rviz::ColorProperty>(
        "Truck", QColor(0, 0, 70), "Truck coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_bus_ = std::make_unique<rviz::ColorProperty>(
        "Bus", QColor(0, 60, 100), "Bus coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_train_ = std::make_unique<rviz::ColorProperty>(
        "Train", QColor(0, 80, 100), "Train coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_motor_cycle_ = std::make_unique<rviz::ColorProperty>(
        "MotorCycle", QColor(0, 0, 230), "MotorCycle coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_bicycle_ = std::make_unique<rviz::ColorProperty>(
        "Bicycle", QColor(119, 11, 32), "Bicycle coloring.", prop_visualization_.get(), SLOT(update()), this);
    prop_coloring_unknown_ = std::make_unique<rviz::ColorProperty>(
        "Unknown", QColor(128, 128, 128), "Unknown coloring.", prop_visualization_.get(), SLOT(update()), this);

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
}

void ObjectStateArrayDisplay::updateDropdown() {
    switch (prop_dropdown_visu_->getOptionInt()) {
    case 1: ///< primitives
        params_visual_.make_mesh = false;
        params_visual_.make_primitive = true;
        break;

    case 2: ///< mesh
        params_visual_.make_mesh = true;
        params_visual_.make_primitive = false;
        break;

    default:
        params_visual_.make_mesh = true;
        params_visual_.make_primitive = false;
    }
}

void ObjectStateArrayDisplay::updateColoring() {
    prop_coloring_road_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_sidewalk_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_building_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_wall_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_fence_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_pole_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_traffic_light_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_traffic_sign_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_vegetation_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_terrain_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_sky_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_person_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_rider_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_car_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_truck_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_bus_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_train_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_motor_cycle_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_bicycle_->setHidden(!prop_coloring_by_class_->getBool());
    prop_coloring_unknown_->setHidden(!prop_coloring_by_class_->getBool());
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

    updateDropdown();
}


Ogre::ColourValue ObjectStateArrayDisplay::colorFromClassification(
    const automated_driving_msgs::ObjectClassification& classification) {

    Ogre::ColourValue color;

    if (!prop_coloring_by_class_->getBool() || classification.semantic_classes_with_probabilities.empty()) {
        color = prop_coloring_unknown_->getOgreColor();
    } else {
        switch (classification.semantic_classes_with_probabilities.front().classification) {

        case automated_driving_msgs::ObjectClassification::ROAD:
            color = prop_coloring_road_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::SIDEWALK:
            color = prop_coloring_sidewalk_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::BUILDING:
            color = prop_coloring_building_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::WALL:
            color = prop_coloring_wall_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::FENCE:
            color = prop_coloring_fence_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::POLE:
            color = prop_coloring_pole_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::TRAFFICLIGHT:
            color = prop_coloring_traffic_light_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::TRAFFICSIGN:
            color = prop_coloring_traffic_sign_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::VEGETATION:
            color = prop_coloring_vegetation_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::TERRAIN:
            color = prop_coloring_terrain_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::SKY:
            color = prop_coloring_sky_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::PERSON:
            color = prop_coloring_person_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::RIDER:
            color = prop_coloring_rider_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::CAR:
            color = prop_coloring_car_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::TRUCK:
            color = prop_coloring_truck_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::BUS:
            color = prop_coloring_bus_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::TRAIN:
            color = prop_coloring_train_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::MOTORCYCLE:
            color = prop_coloring_motor_cycle_->getOgreColor();
            break;
        case automated_driving_msgs::ObjectClassification::BICYCLE:
            color = prop_coloring_bicycle_->getOgreColor();
            break;

        default:
            color = prop_coloring_unknown_->getOgreColor();
        }
    }
    color.a = prop_coloring_alpha_->getFloat();
    return color;
}

void ObjectStateArrayDisplay::processMessage(const Msg::ConstPtr& msg) {

    msg_last_ = msg;
    bool transforms_ok = true;

    updateParameters();
    visuals_.clear();

    if (msg == nullptr || messages_received_ == 0) {
        setStatusStd(rviz::StatusProperty::Warn, "Topic", "No message received");
        return;
    }

    if ((msg->header.stamp - ros::Time::now()).toSec() > 1.0) {
        setStatusStd(rviz::StatusProperty::Warn, "Topic", "Message delay > 1 second");
    }

    visuals_.reserve(msg->objects.size());
    for (const auto& obj : msg->objects) {
        params_visual_.color = colorFromClassification(obj.classification);
        /**
         * Get transform
         */
        Ogre::Quaternion orientation;
        Ogre::Vector3 position;
        if (!context_->getFrameManager()->transform(
                obj.motion_state.header, obj.motion_state.pose.pose, position, orientation)) {
            setStatusStd(rviz::StatusProperty::Error,
                         "Frame",
                         "No transformation found for frame '" + obj.motion_state.header.frame_id + "'");
            transforms_ok = false;
        }
        if (position.isNaN() || orientation.isNaN()) {
            setStatusStd(rviz::StatusProperty::Error, "Frame", "Motion state contains NANs");
            transforms_ok = false;
            continue;
        }
        visuals_.emplace_back(
            context_->getSceneManager(), scene_node_->createChildSceneNode(position, orientation), obj, params_visual_);
    }

    if (transforms_ok)
        setStatusStd(rviz::StatusProperty::Ok, "Frame", "'" + msg->header.frame_id + "'");
}

} // namespace object_state_array_rviz_plugin_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(object_state_array_rviz_plugin_ros::ObjectStateArrayDisplay, rviz::Display)
