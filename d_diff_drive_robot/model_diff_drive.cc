#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>


// For Ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include <ach.h>
#include <string.h>

// ach channels
ach_channel_t chan_diff_drive_ref;      // hubo-ach

int debug = 0;
double H_ref[2] = {};

namespace gazebo
{   
  class ModelDiffDrive : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {

      // Open Ach channel
        /* open ach channel */
        memset( &H_ref,   0, sizeof(H_ref));
        int r = ach_open(&chan_diff_drive_ref, "robot-diff-drive" , NULL);
        assert( ACH_OK == r );
        ach_put(&chan_diff_drive_ref, &H_ref , sizeof(H_ref));

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

      // Get Ach chan data
    size_t fs;
    int r = ach_get( &chan_diff_drive_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );

    if(ACH_OK != r | ACH_STALE_FRAMES != r | ACH_MISSED_FRAME != r) {
                if(debug) {
                  //      printf("Ref ini r = %s\n",ach_result_to_string(r));}
                   printf("ref int r = %i \n\r",r);
		 }
        }
        else{   assert( sizeof(H_ref) == fs ); }


//    if(ACH_OK != r  || ACH_STALE_FRAMES != r || ACH_MISSED_FRAME != r) {
//      fprintf(stderr, "Ref r %i ",r);
//    }
//    else{
//      assert( sizeof(H_ref) == fs);

      this->right_wheel_joint_->SetForce(0, H_ref[0]);
      this->left_wheel_joint_->SetForce(0, H_ref[1]);
//    }

     //this->left_wheel_joint_->SetMaxForce(0, 10);
      //this->right_wheel_joint_->SetMaxForce(0, 10);
      //this->left_wheel_joint_->SetForce(0, -0.5);
//      this->left_wheel_joint_->SetForce(0, 0.2);
      //this->right_wheel_joint_->SetVelocity(0,0.5);
//      this->right_wheel_joint_->SetForce(0, -0.2);

      

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
