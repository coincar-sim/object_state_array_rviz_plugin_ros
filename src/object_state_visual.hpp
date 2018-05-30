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

#pragma once

#include <memory>
#include <automated_driving_msgs/ObjectState.h>
#include <util_automated_driving_msgs/util_automated_driving_msgs.hpp>

#ifndef Q_MOC_RUN
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <util_rviz/util_rviz.hpp>
#include <util_rviz/util_rvizshapes.hpp>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/mesh_shape.h>
#include <rviz/ogre_helpers/movable_text.h>
#endif

namespace object_state_array_rviz_plugin_ros {

struct ObjectStateVisual {

    struct Parameters {
        bool make_arrow{true};
        bool make_mesh{false};
        bool make_text{true};
        bool make_primitive{true};


        Ogre::ColourValue color{Ogre::ColourValue::Black};

        float arrow_v_min{1};
        float arrow_v_max{10};
        float arrow_length{1};

        float text_font_size{1};
        float text_v_min{1};
        bool text_debug{false};
    };

    using Msg = automated_driving_msgs::ObjectState;

    ObjectStateVisual(Ogre::SceneManager*, Ogre::SceneNode*, const Msg&, const Parameters&);
    virtual inline ~ObjectStateVisual() {
        scene_manager_->destroySceneNode(scene_node_);
    }

private:
    void makeArrow(Ogre::SceneNode*);
    void makeMesh(Ogre::SceneNode*);
    void makeText(Ogre::SceneNode*);
    void makePrimitive(Ogre::SceneNode*);

    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* scene_node_; ///< Environment frame

    std::shared_ptr<rviz::MeshShape> mesh_shape_;       ///< Surface mesh
    std::shared_ptr<rviz::Arrow> arrow_;                ///< Velocity arrow
    std::shared_ptr<rviz::MovableText> movable_text_;   ///< Info text
    std::shared_ptr<rviz::MultiShape> primitive_shape_; ///< Object primitive

    Msg obj_;
    Parameters params_;
};

} // namespace object_state_array_rviz_plugin_ros
