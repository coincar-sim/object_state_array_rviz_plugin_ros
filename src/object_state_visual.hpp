#pragma once

#include <memory>
#include <automated_driving_msgs/ObjectState.h>
#include <simulation_utils/util_perception.hpp>

#ifndef Q_MOC_RUN
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <simulation_utils/util_rviz.hpp>
#include <simulation_utils/util_rvizshapes.hpp>
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
