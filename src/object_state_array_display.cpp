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
