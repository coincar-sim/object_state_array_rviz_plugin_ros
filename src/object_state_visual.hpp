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

#ifndef object_state_visual_H
#define object_state_visual_H

#include <automated_driving_msgs/ObjectStateArray.h>

namespace Ogre {
class Vector3;
class Quaternion;
}

namespace rviz {
class Arrow;
class Shape;
class MovableText;
}

namespace object_state_array_rviz_plugin_ros {

// BEGIN_TUTORIAL
// Declare the visual class for this display.
//
// Each instance of ObjectStateArrayVisual represents the visualization of a single
// automated_driving_msgs::ObjectStateArray message.  Currently it just shows an arrow with
// the direction and magnitude of the velocity vector, but could
// easily be expanded to include more of the message data.
class ObjectStateArrayVisual {
public:
    // Constructor.  Creates the visual stuff and puts it into the
    // scene, but in an unconfigured state.
    ObjectStateArrayVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, Ogre::String childFrameId);
    // Destructor.  Removes the visual stuff from the scene.
    virtual ~ObjectStateArrayVisual();

    // Configure the visual to show the data in the message.
    void setVelocityVec(const Ogre::Vector3& directionVec);
    void setShapeOrientation(const Ogre::Quaternion& orientation);
    void showChildFrameId(bool show);

    // Set the pose of the coordinate frame the message refers to.
    // These could be done inside setMessage(), but that would require
    // calls to FrameManager and error handling inside setMessage(),
    // which doesn't seem as clean.  This way ObjectStateArrayVisual is only
    // responsible for visualization.
    void setFramePosition(const Ogre::Vector3& position);
    void setFrameOrientation(const Ogre::Quaternion& orientation);

    // Set the color and alpha of the visual, which are user-editable
    // parameters and therefore don't come from the ObjectStateArray message.
    void setColor(float r, float g, float b, float a);

private:
    // int length_of_vectors = 5;
    // The object implementing the actual arrow shape
    boost::shared_ptr<rviz::Arrow> velocity_arrow_;
    boost::shared_ptr<rviz::Shape> lower_cube_;
    boost::shared_ptr<rviz::Shape> upper_cube_;
    boost::shared_ptr<rviz::Shape> wheel_fr_;
    boost::shared_ptr<rviz::Shape> wheel_fl_;
    boost::shared_ptr<rviz::Shape> wheel_br_;
    boost::shared_ptr<rviz::Shape> wheel_bl_;
    boost::shared_ptr<rviz::MovableText> movable_text;


    // A SceneNode whose pose is set to match the coordinate frame of
    // the ObjectStateArray message header.
    Ogre::SceneNode* frame_node_;

    // The SceneManager, kept here only so the destructor can ask it to
    // destroy the ``frame_node_``.
    Ogre::SceneManager* scene_manager_;
};
// END_TUTORIAL

} // end namespace object_state_array_rviz_plugin_ros

#endif // object_state_visual_H
