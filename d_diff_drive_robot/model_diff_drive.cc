#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{   
  class ModelDiffDrive : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {

      // Store the pointer to the model
      this->model = _parent;

      // Load parameters for this plugin
      if (this->LoadParams(_sdf))
      {
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ModelDiffDrive::OnUpdate, this));
      }
    }

    public: bool LoadParams(sdf::ElementPtr _sdf) 
    {
      if (this->FindJointByParam(_sdf, this->left_wheel_joint_,
                             "left_wheel_hinge") &&
          this->FindJointByParam(_sdf, this->right_wheel_joint_,
                             "right_wheel_hinge"))
        return true;
      else
        return false;
    }

    public: bool FindJointByParam(sdf::ElementPtr _sdf,
                                  physics::JointPtr &_joint,
                                  std::string _param)
    {
      if (!_sdf->HasElement(_param))
      {
        gzerr << "param [" << _param << "] not found\n";
        return false;
      }
      else
      {
        _joint = this->model->GetJoint(
          _sdf->GetElement(_param)->GetValueString());

        if (!_joint)
        {
          gzerr << "joint by name ["
                << _sdf->GetElement(_param)->GetValueString()
                << "] not found in model\n";
          return false;
        }
      }
      return true;
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      //this->left_wheel_joint_->SetMaxForce(0, 10);
      //this->right_wheel_joint_->SetMaxForce(0, 10);
      //this->left_wheel_joint_->SetForce(0, -0.5);
      this->left_wheel_joint_->SetForce(0, 0.2);
      //this->right_wheel_joint_->SetVelocity(0,0.5);
      this->right_wheel_joint_->SetForce(0, -0.2);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::JointPtr left_wheel_joint_;
    private: physics::JointPtr right_wheel_joint_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelDiffDrive)
}
