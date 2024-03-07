#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
    class MovePlugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            model = _model;

            if (!_sdf->HasElement("left_joint"))
                gzerr << "MovePlugin plugin missing <left_joint> element\n";

            if (!_sdf->HasElement("right_joint"))
                gzerr << "MovePlugin plugin missing <right_joint> element\n";

            leftJoint = _model->GetJoint(_sdf->GetElement("left_joint")->Get<std::string>());
            rightJoint = _model->GetJoint(_sdf->GetElement("right_joint")->Get<std::string>());

            if (!leftJoint)
                gzerr << "Unable to find left joint[" << _sdf->GetElement("left_joint")->Get<std::string>() << "]\n";
            if (!rightJoint)
                gzerr << "Unable to find right joint[" << _sdf->GetElement("right_joint")->Get<std::string>() << "]\n";
                
            // Set the target position
            targetPosition = ignition::math::Vector3d(8.0, 17.0, 0); // Cambia la posición final según sea necesario
  
            // Listen to the update event. This event is broadcast every simulation iteration.
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MovePlugin::OnUpdate, this));
        }

        // Called by the world update start event
        void OnUpdate()
        {
            // Get current pose
            ignition::math::Pose3d pose = model->WorldPose();
            
            // Calculate direction towards the target position
            ignition::math::Vector3d direction = targetPosition - pose.Pos();
            double distanceToTarget = direction.Length();
            direction.Normalize();

            // Calculate the desired orientation towards the target position
            double targetYaw = atan2(direction.Y(), direction.X());
            double currentYaw = pose.Rot().Yaw();
            double yawError = targetYaw - currentYaw;

            //Check if target has been reached
            if (distanceToTarget < 1.0)  {
                leftJoint->SetVelocity(0, 0);
                rightJoint->SetVelocity(0, 0);
                printf("Objetivo alcanzado\n");
        
            // Check if within orientation tolerance
            } else if (std::abs(yawError) > 0.05) {
                // Apply proportional control to adjust the orientation
                leftJoint->SetVelocity(0, 0);
                rightJoint->SetVelocity(0, 0);
                double maxAngularVelocity = 1.0; // Ajusta según sea necesario
                double angularVelocity = std::min(std::max(yawError, -maxAngularVelocity), maxAngularVelocity);
                printf("At: %f %f %f\n %f \n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), distanceToTarget);

                // Set the angular velocity
                model->SetAngularVel(ignition::math::Vector3d(0, 0, angularVelocity));
            } else {
                // Within orientation tolerance, stop angular velocity
                model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
                // Set the linear velocity
                leftJoint->SetVelocity(0, 2.0);
                rightJoint->SetVelocity(0, 2.0);
                printf("At: %f %f %f\n %f \n", pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), distanceToTarget);
            }
           
        }

    private:
        physics::ModelPtr model; // Pointer to the model
        physics::JointPtr leftJoint, rightJoint;
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
        ignition::math::Vector3d targetPosition; // Posición final del robot
        
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MovePlugin)
}