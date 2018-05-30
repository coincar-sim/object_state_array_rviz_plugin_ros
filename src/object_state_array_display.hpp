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
#include <automated_driving_msgs/ObjectStateArray.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-parameter"
#include <rviz/message_filter_display.h>
#pragma GCC diagnostic pop
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>

#include "object_state_visual.hpp"

namespace object_state_array_rviz_plugin_ros {

class ObjectStateArrayDisplay : public rviz::MessageFilterDisplay<automated_driving_msgs::ObjectStateArray> {
    Q_OBJECT

    using Msg = automated_driving_msgs::ObjectStateArray;

public:
    ObjectStateArrayDisplay();
    inline virtual ~ObjectStateArrayDisplay(){};

protected:
    inline void reset() override {
        MFDClass::reset();
        visuals_.clear();
        processMessage(msg_last_);
    }

    inline virtual void onEnable() override {
        MFDClass::onEnable();
        processMessage(msg_last_);
    }

    inline virtual void updateTopic() override {
        MFDClass::updateTopic();
        processMessage(msg_last_);
    }

private Q_SLOTS:
    inline void update() {
        processMessage(msg_last_);
    }
    void updateColoring();
    void updateArrows();

    void updateDropdown();

private:
    void updateParameters();
    void processMessage(const Msg::ConstPtr&) override;

    Ogre::ColourValue colorFromClassification(const automated_driving_msgs::ObjectClassification& classification);

    Msg::ConstPtr msg_last_ = nullptr; ///< The last message will be buffered

    std::vector<ObjectStateVisual> visuals_; ///< Container for all object visuals
    ObjectStateVisual::Parameters params_visual_;

    // rviz properties
    std::unique_ptr<rviz::Property> prop_visualization_;
    std::unique_ptr<rviz::EnumProperty> prop_dropdown_visu_;
    std::unique_ptr<rviz::BoolProperty> prop_show_arrows_, prop_text_show_, prop_text_debug_, prop_coloring_by_class_;
    std::unique_ptr<rviz::FloatProperty> prop_coloring_alpha_, prop_arrow_v_min_, prop_arrow_v_max_, prop_arrow_length_,
        prop_text_size_;
    std::unique_ptr<rviz::ColorProperty> prop_coloring_unknown_, prop_coloring_vehicle_, prop_coloring_pedestrian_,
        prop_coloring_bike_;
};

} // namespace object_state_array_rviz_plugin_ros
