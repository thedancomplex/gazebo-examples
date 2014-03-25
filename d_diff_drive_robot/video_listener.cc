/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <iostream>


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

// ach channels
ach_channel_t chan_vid_chan;      // hubo-ach


int i = 0;
/////////////////////////////////////////////////
// Function is called everytime a message is received.
//void cb(gazebo::msgs::Image &_msg)
//void cb(const std::string& _msg)
//void cb(gazebo::msgs::ImageStamped &_msg)
//void cb(ConstWorldStatisticsPtr &_msg)
//void cb(const std::string& _msg)
void cb(ConstImageStampedPtr &_msg)
{
  // Dump the message contents to stdout.
// char filename[256];

//  gazebo::msgs::ImageStamped _img = _msg; 

//gazebo::common::Image::PixelFormat img = gazebo::common::Image::ConvertPixelFormat (_msg);

//static Image::PixelFormat 	ConvertPixelFormat (const std::string &_format)
// 	Convert a string to a Image::PixelFormat. 

//std::cout << _msg->DebugString();
  size_t size;
//  const gazebo::common::Image &_img = gazebo::common::Image.begin();
// gazebo::msgs::Set( _msg, _img);


  //int ssize = sizeof(_msg->image().data());
//  int ssize = sizeof(img.data);
///  int ssize = sizeof(_msg.c_str());


 // printf("%i - %i - w: %i - h: %i - format %i \n\r",i++, ssize, _msg->image().width(), _msg->image().height(), _msg->image().height());


//////  printf("%i - %i - %i \n\r",i++, ssize, _msg->image().data().size() );



// char filename[256];
 ach_put(&chan_vid_chan, _msg->image().data().c_str() , _msg->image().data().size());

  // Save frames
  /*if (this->save)
  {
    snprintf(filename, sizeof(filename), "click-%04d.ppm", this->frameno++);
    this->SaveFrame(filename);
  }*/
  // Save frames
  /*if (this->save)
  {
    snprintf(filename, sizeof(filename), "click-%04d.ppm", this->frameno++);
    this->SaveFrame(filename);
  }*/



}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
        //gazebo::transport::clear_buffers();
        char robot_vid[320*240*3];
	memset( &robot_vid,   0, sizeof(robot_vid));


        /* open ach channel */
	int r = ach_open(&chan_vid_chan, "robot-vid-chan" , NULL);
	assert( ACH_OK == r );



  // Load gazebo
  printf("%i\n\r",-1);
  gazebo::load(_argc, _argv);

  printf("%i\n\r",0);
  gazebo::run();

  printf("%i\n\r",1);
  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  printf("%i\n\r",2);
  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/DiffDrive/d_diff_drive_robot/camera/link/camera/image", cb);
//  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/DiffDrive/d_diff_drive_robot/camera/link/camera/image", cb);

  printf("%i\n\r",3);
  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(100);

  // Make sure to shut everything down.
  gazebo::transport::fini();
}
