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

#include "object_state_visual.hpp"

#include <rviz/msg_conversions.h>
#include <util_rviz/util_rviz.hpp>
#include <util_rviz/util_rvizshapes.hpp>

namespace object_state_array_rviz_plugin_ros {

ObjectStateVisual::ObjectStateVisual(Ogre::SceneManager* sm, Ogre::SceneNode* sn, const Msg& obj, const Parameters& p)
        : scene_manager_{sm}, scene_node_{sn}, obj_{obj}, params_(p) {

    if (params_.make_mesh)
        makeMesh(scene_node_);

    if (params_.make_primitive)
        makePrimitive(scene_node_);

    if (params_.make_arrow)
        makeArrow(scene_node_);

    if (params_.make_text)
        makeText(scene_node_);
}


void ObjectStateVisual::makePrimitive(Ogre::SceneNode* sn) {
    if (obj_.classification.classes_with_probabilities.empty()) {
        primitive_shape_ = std::make_shared<rviz::SimpleUnknown>(scene_manager_, sn);
    } else {
        switch (obj_.classification.classes_with_probabilities.front().classification) {

        case automated_driving_msgs::ObjectClassification::CAR:
            primitive_shape_ = std::make_shared<rviz::SimpleCar>(scene_manager_, sn);
            break;

        case automated_driving_msgs::ObjectClassification::PEDESTRIAN:
            primitive_shape_ = std::make_shared<rviz::SimplePedestrian>(scene_manager_, sn);
            break;

        case automated_driving_msgs::ObjectClassification::BICYCLE:
            primitive_shape_ = std::make_shared<rviz::SimpleBike>(scene_manager_, sn);
            break;

        default:
            primitive_shape_ = std::make_shared<rviz::SimpleUnknown>(scene_manager_, sn);
        }
    }

    primitive_shape_->setColor(params_.color);
}

void ObjectStateVisual::makeMesh(Ogre::SceneNode* sn) {

    mesh_shape_ = std::make_shared<rviz::MeshShape>(scene_manager_, sn);
    mesh_shape_->setColor(params_.color);

    mesh_shape_->estimateVertexCount(obj_.hull.vertices.size());
    for (auto& v : obj_.hull.vertices)
        mesh_shape_->addVertex(rviz::pointMsgToOgre(v));
    for (auto& t : obj_.hull.triangles)
        mesh_shape_->addTriangle(t.vertex_indices[0], t.vertex_indices[1], t.vertex_indices[2]);
    mesh_shape_->endTriangles();
}

void ObjectStateVisual::makeArrow(Ogre::SceneNode* sn) {

    if (!util_automated_driving_msgs::checks::twistValid(obj_.motion_state)) {
        return;
    }

    const Ogre::Vector3 v{rviz::vector3MsgToOgre(obj_.motion_state.twist.twist.linear)};
    if (v.isNaN()) {
        return;
    }

    arrow_ = std::make_shared<rviz::Arrow>(scene_manager_, sn);
    arrow_->setColor(params_.color);
    arrow_->setDirection(v);

    const double velocity{v.length()};
    double scale;

    if (velocity > params_.arrow_v_max)
        scale = params_.arrow_length;
    else if (velocity < params_.arrow_v_min)
        scale = 0.0;
    else
        scale = params_.arrow_length * (velocity - params_.arrow_v_min) / (params_.arrow_v_max - params_.arrow_v_min);

    arrow_->setScale(Ogre::Vector3(scale, 1, 1));
}

void ObjectStateVisual::makeText(Ogre::SceneNode* sn) {

    std::ostringstream os;
    os << "ID " << std::to_string(obj_.object_id);

    auto const& v = obj_.motion_state.twist.twist.linear;
    const double velocity{sqrt(v.x * v.x + v.y * v.y + v.z * v.z)};
    if (velocity > params_.text_v_min || params_.text_debug)
        os << "\n"
           << std::to_string(static_cast<int>(3.6 * velocity)) << " km/h"
           << "\n";

    if (params_.text_debug) {
        /**
         * Additional debug info here
         */
        auto const& cp = obj_.motion_state.pose.covariance;
        const double cov{cp[0] * cp[0] + cp[7] * cp[7] + cp[14] * cp[14]};
        os << "\nCOV " << cov;
    }

    movable_text_ = std::make_shared<rviz::MovableText>(os.str());
    movable_text_->setCharacterHeight(params_.text_font_size);
    movable_text_->setTextAlignment(rviz::MovableText::HorizontalAlignment::H_LEFT,
                                    rviz::MovableText::VerticalAlignment::V_ABOVE);
    movable_text_->showOnTop();
    sn->attachObject(movable_text_.get());
}

} // namespace object_state_array_rviz_plugin_ros
