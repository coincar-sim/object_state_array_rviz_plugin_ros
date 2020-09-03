#pragma once

#include <automated_driving_msgs/ObjectState.h>

#ifndef Q_MOC_RUN
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <util_rviz/util_rviz.hpp>
#include <util_rviz/util_rvizshapes.hpp>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/status_property.h>
#endif

namespace object_state_array_rviz_plugin_ros {

struct ObjectPredictionVisual {

    /// @todo: share struct with state visual
    struct Status {
        using Level = rviz::StatusProperty::Level;

        inline Status(const unsigned id) : message{"| Id " + std::to_string(id) + ": "} {
        }

        Level level{Level::Ok};
        std::string message;
    };

    struct Parameters {

        Ogre::ColourValue color{Ogre::ColourValue::Black};

        float size{0.05f};
        float width{0.5f};

        bool prediction_enabled{false};

        double time_horizon{3.0};

        ulong max_num_trajectories{1};

        bool centerline{true};
        bool corridor{false};
    };

    using Msg = automated_driving_msgs::ObjectState;

    ObjectPredictionVisual(Ogre::SceneManager*, Ogre::SceneNode*, const Msg&, const Parameters&);

    virtual inline ~ObjectPredictionVisual() {
        scene_manager_->destroySceneNode(scene_node_);
    }

    inline const Status& getStatus() const {
        return status_;
    }


public:
private:
    void generateCorridor(const Ogre::Vector3& previous,
                          const Ogre::Vector3& current,
                          std::vector<rviz::PointCloud::Point>& points,
                          float min_y,
                          float max_y);
    void makePrediction(Ogre::SceneNode*);

    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* scene_node_; ///< Environment frame

    std::vector<std::shared_ptr<rviz::BillboardLine>> prediction_center_;     ///< Prediction center
    std::vector<std::shared_ptr<rviz::BillboardLine>> prediction_line_left_;  ///< Prediction left border
    std::vector<std::shared_ptr<rviz::BillboardLine>> prediction_line_right_; ///< Prediction right border
    std::shared_ptr<rviz::PointCloud> prediction_point_cloud_;                ///< Point cloud

    Msg obj_;
    Parameters params_;
    Status status_;
};
} // namespace object_state_array_rviz_plugin_ros
