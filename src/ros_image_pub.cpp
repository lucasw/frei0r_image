/* ros_image_pub
 * Copyright 2019 Lucas Walter
 * This file is a Frei0r plugin.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <algorithm>
#include <assert.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <memory>
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stdlib.h>
#include <string>

// This makes the library symbols not mangled,
// e.g. 0000000000007dc0 T f0r_deinit instead of
// 0000000000007eb0 T _Z10f0r_deinitv
extern "C" {
  #include <frei0r.h>
}

typedef struct ros_image_pub_instance {
  unsigned int width_;
  unsigned int height_;
  std::string topic_ = "image_out";
  std::string frame_id_ = "frei0r";

  bool initted_ = false;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Publisher pub_;

  void update(const unsigned char* inframe)
  {
    if (!nh_) {
      if (!ros::master::check()) {
        return;
      }
      int argc = 0;
      // TODO(lucasw) this can go back in construct
      ros::init(argc, nullptr, "frei0r", ros::init_options::AnonymousName);
      ROS_INFO_STREAM("initialized ros node");
      nh_ = std::make_unique<ros::NodeHandle>();
      pub_ = nh_->advertise<sensor_msgs::Image>(topic_, 3);
    }
    sensor_msgs::Image image_out;
    image_out.header.stamp = ros::Time::now();
    image_out.header.frame_id = frame_id_;
    image_out.width = width_;
    image_out.height = height_;
    image_out.step = width_ * 4;
    image_out.encoding = "bgra8";
    image_out.data.resize(image_out.step * height_);
    std::copy(inframe, inframe + image_out.data.size(), &image_out.data[0]);
    pub_.publish(image_out);
    ros::spinOnce();
  }
} ros_image_pub_instance_t;

int f0r_init() {
  int argc = 0;
  ROS_INFO_STREAM("ros_image_pub init " << ros::this_node::getName());
  return 1;
}

void f0r_deinit() {
  ROS_INFO_STREAM("deinit");
}

void f0r_get_plugin_info(f0r_plugin_info_t *inverterInfo) {
  inverterInfo->name = "ros_image_pub";
  inverterInfo->author = "Lucas Walter";
  inverterInfo->plugin_type = F0R_PLUGIN_TYPE_FILTER;
  inverterInfo->color_model = F0R_COLOR_MODEL_BGRA8888;
  inverterInfo->frei0r_version = FREI0R_MAJOR_VERSION;
  inverterInfo->major_version = 0;
  inverterInfo->minor_version = 1;
  inverterInfo->num_params = 1;
  inverterInfo->explanation = "publishes to a ros image topic";
}

void f0r_get_param_info(f0r_param_info_t *info, int param_index) {
  switch (param_index) {
  case 0:
    info->name = "ros image topic";
    info->type = F0R_PARAM_STRING;
    info->explanation = "ros image topic";
    break;
  }
}

f0r_instance_t f0r_construct(unsigned int width, unsigned int height) {
  ros_image_pub_instance_t *inst = new ros_image_pub_instance;

  inst->width_ = width;
  inst->height_ = height;

  ROS_INFO_STREAM("ros_image_pub construct " << ros::this_node::getName() << " "
      << width << " x " << height << " " << inst->topic_);
  return (f0r_instance_t)inst;
}

void f0r_destruct(f0r_instance_t instance) {
  std::cout << "destruct\n";
  ros_image_pub_instance_t *inst = reinterpret_cast<ros_image_pub_instance_t*>(instance);
  delete inst;
}

void f0r_set_param_value(f0r_instance_t instance, f0r_param_t param,
                         int param_index) {
  assert(instance);
  ros_image_pub_instance_t *inst = reinterpret_cast<ros_image_pub_instance_t*>(instance);

  switch (param_index) {
  case 0:
    const std::string topic = std::string(*(reinterpret_cast<char**>(param)));
    if (topic != inst->topic_) {
      inst->topic_ = topic;
      ROS_INFO_STREAM("new topic " << inst->topic_);
      if (inst->nh_) {
        inst->pub_.shutdown();
        if (inst->topic_ != "") {
          inst->pub_ = inst->nh_->advertise<sensor_msgs::Image>(inst->topic_, 3);
        }
      }
    }
    break;
  }
}

void f0r_get_param_value(f0r_instance_t instance, f0r_param_t param,
                         int param_index) {
  assert(instance);
  ros_image_pub_instance_t *inst = reinterpret_cast<ros_image_pub_instance_t*>(instance);

  switch (param_index) {
  case 0:
    ROS_INFO_STREAM("get param start");
    *(reinterpret_cast<f0r_param_string*>(param)) = const_cast<char*>(inst->topic_.data());
    ROS_INFO_STREAM("get param done");
    break;
  }
}

void f0r_update(f0r_instance_t instance, double time, const uint32_t *inframe,
                uint32_t *outframe) {
  assert(instance);
  ros_image_pub_instance_t *inst = reinterpret_cast<ros_image_pub_instance_t*>(instance);

  // TODO(lucasw) there is no frei0r 'sink' type, so need to do something with the output.
  // TODO(lucasw) should this be a copy instead?
  // outframe = inframe;
  inst->update(reinterpret_cast<const unsigned char *>(inframe));
}
