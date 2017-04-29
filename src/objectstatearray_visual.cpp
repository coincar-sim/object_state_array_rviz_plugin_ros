/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/movable_text.h>
#include <rviz/ogre_helpers/shape.h>

#include "objectstatearray_visual.hpp"

namespace object_state_array_rviz_plugin_ros {

ObjectStateArrayVisual::ObjectStateArrayVisual(Ogre::SceneManager* scene_manager,
                                               Ogre::SceneNode* parent_node,
                                               Ogre::String childFrameId) {
    scene_manager_ = scene_manager;

    // Ogre::SceneNode s form a tree, with each node storing the
    // transform (position and orientation) of itself relative to its
    // parent.  Ogre does the math of combining those transforms when it
    // is time to render.
    //
    // Here we create a node to store the pose of the ObjectStateArray's header frame
    // relative to the RViz fixed frame.
    frame_node_ = parent_node->createChildSceneNode();
    // We create the arrow object within the frame node so that we can
    // set its position and direction relative to its header frame.

    float wheelDiameter = 0.8;
    float wheelWidth = 0.4;
    Ogre::Vector3 wheelScale(wheelDiameter, wheelWidth, wheelDiameter);
    Ogre::Vector3 wheelPosition(0.0, 0.0, wheelScale.z / 2.0);

    lower_cube_.reset(new rviz::Shape(rviz::Shape::Cube, scene_manager_, frame_node_));
    Ogre::Vector3 lowerCubeScale(4, 1.8, 0.8);
    lower_cube_->setScale(lowerCubeScale);
    Ogre::Vector3 lowerCubePosition(0, 0, lowerCubeScale.z / 2.0 + wheelScale.z / 2.0);
    lower_cube_->setPosition(lowerCubePosition);

    upper_cube_.reset(new rviz::Shape(rviz::Shape::Cube, scene_manager_, frame_node_));
    Ogre::Vector3 upperCubeScale(2, 1.8, 0.8);
    upper_cube_->setScale(upperCubeScale);
    Ogre::Vector3 upperCubePosition(0, 0, lowerCubeScale.z + upperCubeScale.z / 2.0 + wheelScale.z / 2.0);
    upper_cube_->setPosition(upperCubePosition);

    velocity_arrow_.reset(new rviz::Arrow(scene_manager_, frame_node_));
    Ogre::Vector3 arrowScale(0, 0, 0);
    velocity_arrow_->setScale(arrowScale);
    Ogre::Vector3 arrowPosition(0, 0, lowerCubeScale.z / 2.0);
    velocity_arrow_->setPosition(arrowPosition);

    float lengthCenterToAxisX = 1.4/wheelScale.x;
    float lengthCenterToWheelY = lowerCubeScale.y/2.0/wheelScale.y;

    wheel_fr_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, frame_node_));
    wheel_fr_->setPosition(wheelPosition);
    wheel_fr_->setColor(0.0, 0.0, 0.0, 1.0);
    wheel_fr_->setScale(wheelScale);
    Ogre::Vector3 wheelFROffset(lengthCenterToAxisX, -lengthCenterToWheelY, 0.0);
    wheel_fr_->setOffset(wheelFROffset);

    wheel_fl_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, frame_node_));
    wheel_fl_->setPosition(wheelPosition);
    wheel_fl_->setColor(0.0, 0.0, 0.0, 1.0);
    wheel_fl_->setScale(wheelScale);
    Ogre::Vector3 wheelFLOffset(lengthCenterToAxisX, lengthCenterToWheelY, 0.0);
    wheel_fl_->setOffset(wheelFLOffset);

    wheel_br_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, frame_node_));
    wheel_br_->setPosition(wheelPosition);
    wheel_br_->setColor(0.0, 0.0, 0.0, 1.0);
    wheel_br_->setScale(wheelScale);
    Ogre::Vector3 wheelBROffset(-lengthCenterToAxisX, -lengthCenterToWheelY, 0.0);
    wheel_br_->setOffset(wheelBROffset);

    wheel_bl_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, frame_node_));
    wheel_bl_->setPosition(wheelPosition);
    wheel_bl_->setColor(0.0, 0.0, 0.0, 1.0);
    wheel_bl_->setScale(wheelScale);
    Ogre::Vector3 wheelBLOffset(-lengthCenterToAxisX, lengthCenterToWheelY, 0.0);
    wheel_bl_->setOffset(wheelBLOffset);

    movable_text.reset(new rviz::MovableText(childFrameId, "Arial", 3.0));
    movable_text->setTextAlignment(rviz::MovableText::HorizontalAlignment::H_CENTER,
                                   rviz::MovableText::VerticalAlignment::V_ABOVE);
    movable_text->showOnTop(true);
    frame_node_->attachObject(movable_text.get());
}

ObjectStateArrayVisual::~ObjectStateArrayVisual() {
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode(frame_node_);
}

// Shows movable_text depending on label_property_ "show childFrameId"
void ObjectStateArrayVisual::showChildFrameId(bool show) {
    movable_text->setVisible(show);
}


void ObjectStateArrayVisual::setVelocityVec(const Ogre::Vector3& directionVec) {

    // Set the magnitude of the velocity vector.
    float length;
    if (directionVec.length() > 0) {
        length = (directionVec.length() + 75.0) / 30.0;
    } else {
        length = 0.0;
    }


    // Scale the arrow's thickness in each dimension.
    Ogre::Vector3 scale(length, length, length);
    velocity_arrow_->setScale(scale);

    // Set the orientation of the arrow to match the direction of the
    // acceleration vector.
    velocity_arrow_->setDirection(directionVec);
}

void ObjectStateArrayVisual::setShapeOrientation(const Ogre::Quaternion& orientation) {
    lower_cube_->setOrientation(orientation);
    upper_cube_->setOrientation(orientation);
    wheel_fr_->setOrientation(orientation);
    wheel_fl_->setOrientation(orientation);
    wheel_br_->setOrientation(orientation);
    wheel_bl_->setOrientation(orientation);
}

// Position and orientation are passed through to the SceneNode.
void ObjectStateArrayVisual::setFramePosition(const Ogre::Vector3& position) {
    frame_node_->setPosition(position);
}

void ObjectStateArrayVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
    frame_node_->setOrientation(orientation);
}

// Color is passed through to the Arrow object.
void ObjectStateArrayVisual::setColor(float r, float g, float b, float a) {
    velocity_arrow_->setColor(r, g, b, a);
}


} // end namespace object_state_array_rviz_plugin_ros