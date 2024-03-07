#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{

    class ModelPush : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Store the pointer to the model
            model = _parent;

            // Listen to the update event. This event is broadcast every simulation iteration.
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
        }

        // Called by the world update start event
        void OnUpdate()
        {
            // Apply a small linear velocity to the model.
            model->SetLinearVel(ignition::math::Vector3d(0.1, 0, 0));
            model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
            ignition::math::Pose3d pose = model->WorldPose();
            printf("At: %f %f %f\n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
        }

    private:
        physics::ModelPtr model;               // Pointer to the model
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
