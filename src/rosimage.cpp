/* rosimage
 * binarymillenium 2007
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

#include <assert.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stdlib.h>

extern "C" {
#include "frei0r.h"
}

typedef struct rosimage_instance {
  unsigned int width_;
  unsigned int height_;
  std::string topic_ = "image";

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  cv::Mat image_in_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgra8");
    } catch (cv_bridge::Exception& ex) {
      ROS_ERROR_THROTTLE(1.0, "cv bridge exception %s", ex.what());
      return;
    }
    cv::resize(cv_ptr->image, image_in_,
        cv::Size(width_, height_), cv::INTER_NEAREST);
  }
} rosimage_instance_t;

/* Clamps a int32-range int between 0 and 255 inclusive. */
unsigned char CLAMP0255(int32_t a) {
  return (unsigned char)((((-a) >> 31) & a) // 0 if the number was negative
                         | (255 - a) >>
                               31); // -1 if the number was greater than 255
}

int f0r_init() {
  int argc = 0;
  // ros::init(argc, nullptr, "frei0r", ros::init_options::AnonymousName);
  ROS_INFO_STREAM("rosimage init " << ros::this_node::getName());
  return 1;
}

void f0r_deinit() {}

void f0r_get_plugin_info(f0r_plugin_info_t *inverterInfo) {
  inverterInfo->name = "rosimage";
  inverterInfo->author = "Lucas Walter";
  inverterInfo->plugin_type = F0R_PLUGIN_TYPE_SOURCE;
  inverterInfo->color_model = F0R_COLOR_MODEL_BGRA8888;
  inverterInfo->frei0r_version = FREI0R_MAJOR_VERSION;
  inverterInfo->major_version = 0;
  inverterInfo->minor_version = 2;
  inverterInfo->num_params = 1;
  inverterInfo->explanation = "subscribes to a ros topic";
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
  rosimage_instance_t *inst =
      (rosimage_instance_t *)malloc(sizeof(rosimage_instance_t));

  inst->width_ = width;
  inst->height_ = height;

  ROS_INFO_STREAM("rosimage construct " << ros::this_node::getName() << " "
      << width << " x " << height << " " << inst->topic_);
  // inst->sub_ = inst->nh_.subscribe<sensor_msgs::Image>(inst->topic_, 3,
  //     &rosimage_instance::imageCallback, inst);
  ROS_INFO_STREAM("subscribed");
  return (f0r_instance_t)inst;
}

void f0r_destruct(f0r_instance_t instance) {
  // TODO(lucasw) close the display?
  free(instance);
}

void f0r_set_param_value(f0r_instance_t instance, f0r_param_t param,
                         int param_index) {
  assert(instance);
  rosimage_instance_t *inst = (rosimage_instance_t *)instance;

  // TODO(lucasw) clamp param to 0.0 - 1.0

  switch (param_index) {
  case 0:
    // inst->topic_ = std::string(*(char**)param);
    ROS_INFO_STREAM("new topic " << inst->topic_);
#if 0
    inst->sub_.shutdown();
    inst->sub_ = inst->nh_.subscribe<sensor_msgs::Image>(inst->topic_, 3,
        &rosimage_instance::imageCallback, inst);
#endif
    break;
  }
}

void f0r_get_param_value(f0r_instance_t instance, f0r_param_t param,
                         int param_index) {
  assert(instance);
  rosimage_instance_t *inst = (rosimage_instance_t *)instance;

  switch (param_index) {
  case 0:
    ROS_INFO_STREAM("get param start");
    *((f0r_param_string *)param) = const_cast<char*>(inst->topic_.data());
    ROS_INFO_STREAM("get param done");
    break;
  }
}

void f0r_update(f0r_instance_t instance, double time, const uint32_t *inframe,
                uint32_t *outframe) {
  assert(instance);
  rosimage_instance_t *inst = (rosimage_instance_t *)instance;

  unsigned char *dst = (unsigned char *)outframe;

  ROS_INFO_STREAM("update");
  if (inst->image_in_.empty()) {
    inst->image_in_ = cv::Mat(cv::Size(inst->width_, inst->height_),
                              CV_8UC4, cv::Scalar(0, 0, 0, 0));
  }

  cv::Mat tmp = inst->image_in_;
  if ((inst->image_in_.cols != inst->width_) ||
      (inst->image_in_.rows != inst->height_)) {
    ROS_WARN_STREAM("size mismatch ");
    cv::resize(inst->image_in_, tmp,
        cv::Size(inst->width_, inst->height_), cv::INTER_NEAREST);
  }

  const size_t num_bytes = inst->width_ * inst->height_ * 4;
  std::copy(&tmp.data[0], &tmp.data[0] + num_bytes, dst);
}
