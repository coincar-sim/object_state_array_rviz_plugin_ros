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

#include <tf/transform_listener.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>
#include <std_msgs/Float64.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>

#include "objectstatearray_display.hpp"
#include "objectstatearray_visual.hpp"

namespace object_state_array_rviz_plugin_ros {

ObjectStateArrayDisplay::ObjectStateArrayDisplay() {

    color_property_[0] = new rviz::ColorProperty(
        "Velocity Arrows", QColor(204, 51, 204), "Color to draw the velocity arrows.", this, SLOT(updateColorAndAlpha()));
    alpha_property_ = new rviz::FloatProperty(
        "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this, SLOT(updateColorAndAlpha()));
    label_property_ = new rviz::BoolProperty(
        "show child_frame_id", true, "Shows child_frame_id if enabled.", this, SLOT(showChildFrameId()));
}

void ObjectStateArrayDisplay::onInitialize() {
    MFDClass::onInitialize();
}

ObjectStateArrayDisplay::~ObjectStateArrayDisplay() {
}

// Clear the visuals by deleting their objects.
void ObjectStateArrayDisplay::reset() {
    MFDClass::reset();
    visuals_.clear();
}

// Shows the Label "child_frame_id" for each visual.
void ObjectStateArrayDisplay::showChildFrameId() {
    bool show = label_property_->getBool();

    for (size_t i = 0; i < visuals_.size(); i++) {
        visuals_[i]->showChildFrameId(show);
    }
}

// Set the current color and alpha values for each visual.
void ObjectStateArrayDisplay::updateColorAndAlpha() {
    float alpha = alpha_property_->getFloat();


    for (size_t i = 0; i < visuals_.size(); i++) {
        Ogre::ColourValue color = color_property_[0]->getOgreColor();
        visuals_[i]->setColor(color.r, color.g, color.b, alpha);
    }
}

// Set the number of past visuals to show.
void ObjectStateArrayDisplay::updateNumberOfVehicle(int number_of_vehicles) {
    visuals_.rset_capacity(number_of_vehicles);
}

// This is our callback to handle an incoming message.
void ObjectStateArrayDisplay::processMessage(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg) {

    // length of the object state array
    int number_of_objects = msg->objects.size();
    updateNumberOfVehicle(number_of_objects);

    for (int i = 0; i < number_of_objects; i++) {
        Ogre::Quaternion orientation;
        Ogre::Vector3 position;

        // Transform the received pose into the fixed_frame
        if (!context_->getFrameManager()->transform(msg->objects[i].motion_state.header.frame_id,
                                                    msg->objects[i].motion_state.header.stamp,
                                                    msg->objects[i].motion_state.pose.pose,
                                                    position,
                                                    orientation)) {
            ROS_WARN("Error transforming from frame '%s' to frame '%s'",
                     msg->objects[i].motion_state.header.frame_id.c_str(),
                     qPrintable(fixed_frame_));
            return;
        }


        // We are keeping a circular buffer of visual pointers.  This gets
        // the next one, or creates and stores it if the buffer is not full
        boost::shared_ptr<ObjectStateArrayVisual> visual;
        if (visuals_.full()) {
            visual = visuals_.front();
        } else {
            visual.reset(new ObjectStateArrayVisual(
                context_->getSceneManager(), scene_node_, msg->objects[i].motion_state.child_frame_id));
        }

        // Now set or update the contents of the chosen visual.
        visual->setFramePosition(position);
        visual->setShapeOrientation(orientation);
        bool show = label_property_->getBool();
        visual->showChildFrameId(show);

        if (msg->objects[i].motion_state.twist.covariance[0] >= 0) {
            const geometry_msgs::Vector3& a = msg->objects[i].motion_state.twist.twist.linear;
            Ogre::Vector3 velocityLinear(a.x, a.y, a.z);
            visual->setVelocityVec(velocityLinear);
        } else {
            Ogre::Vector3 noVelocity(0, 0, 0);
            visual->setVelocityVec(noVelocity);
        }

        float alpha = alpha_property_->getFloat();
        Ogre::ColourValue color = color_property_[0]->getOgreColor();
        visual->setColor(color.r, color.g, color.b, alpha);

        // And send it to the end of the circular buffer
        visuals_.push_back(visual);
    }
}

} // end namespace object_state_array_rviz_plugin_ros

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(object_state_array_rviz_plugin_ros::ObjectStateArrayDisplay, rviz::Display)
