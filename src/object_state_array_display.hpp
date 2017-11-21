#pragma once

#include <memory>
#include <automated_driving_msgs/ObjectStateArray.h>
#include <rviz/message_filter_display.h>
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

    void updp();

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
