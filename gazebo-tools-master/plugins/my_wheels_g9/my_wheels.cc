#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{

    class MyWheels : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            model = _model;

            if (!_sdf->HasElement("left_joint"))
                gzerr << "MyWheels plugin missing <left_joint> element\n";

            if (!_sdf->HasElement("right_joint"))
                gzerr << "MyWheels plugin missing <right_joint> element\n";

            leftJoint = _model->GetJoint(_sdf->GetElement("left_joint")->Get<std::string>());
            rightJoint = _model->GetJoint(_sdf->GetElement("right_joint")->Get<std::string>());

            if (!leftJoint)
                gzerr << "Unable to find left joint[" << _sdf->GetElement("left_joint")->Get<std::string>() << "]\n";
            if (!rightJoint)
                gzerr << "Unable to find right joint[" << _sdf->GetElement("right_joint")->Get<std::string>() << "]\n";

            // Listen to the update event. This event is broadcast every simulation iteration.
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MyWheels::OnUpdate, this));
        }

        // Called by the world update start event
        void OnUpdate()
        {
            leftJoint->SetVelocity(0, 1.0);
            rightJoint->SetVelocity(0, 1.0);
            ignition::math::Pose3d pose = model->WorldPose();
            printf("At: %f %f %f\n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
        }

    private:
        physics::ModelPtr model; // Pointer to the model
        physics::JointPtr leftJoint, rightJoint;
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MyWheels)
}
