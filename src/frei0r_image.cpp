/**
 * Copyright (c) 2019 Lucas Walter
 * Load frei0r plugins following these guidelines:
 * https://frei0r.dyne.org/codedoc/html/group__pluglocations.html
 */

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <experimental/filesystem>
#include <frei0r_image/frei0r_image.hpp>
// TODO(lucasw) there is a C++ header in the latest frei0r sources,
// but it isn't in Ubuntu 18.04 released version currently
// #define _UNIX03_SOURCE
#include <dlfcn.h>
#include <frei0r.h>
#include <frei0r_image/frei0r_image.hpp>
#include <iostream>
#include <map>
#include <nodelet/nodelet.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>


namespace frei0r_image
{

Frei0rImage::Frei0rImage()
{
}

void Frei0rImage::onInit()
{
  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image", 3);

  std::string name = "/usr/lib/frei0r-1/nois0r.so";
  getPrivateNodeHandle().getParam("library", name);
  loadLibrary(name);

  unsigned int wd = 8;
  unsigned int ht = 8;
  setSize(wd, ht);

  ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(getPrivateNodeHandle());
  ddr_->registerVariable<int>("width", 8,
      boost::bind(&Frei0rImage::widthCallback, this, _1), "width", 8, 2048);
  ddr_->registerVariable<int>("height", 8,
      boost::bind(&Frei0rImage::heightCallback, this, _1), "height", 8, 1024);

  for (size_t i = 0; i < fi_.num_params; ++i) {
    // TODO(lucasw) create a control for each parameter
  }

  ddr_->publishServicesTopics();

  timer_ = getPrivateNodeHandle().createTimer(ros::Duration(0.1),
      &Frei0rImage::update, this);
}

bool Frei0rImage::loadLibrary(const std::string& name)
{
  handle_ = dlopen(name.c_str(), RTLD_NOW);
  if (!handle_) {
    return false;
  }
  // std::cout << name << " " << handle_ << "\n";

  init = (f0r_init_t)dlsym(handle_, "f0r_init");
  deinit = (f0r_deinit_t)dlsym(handle_, "f0r_deinit");
  get_plugin_info = (f0r_get_plugin_info_t)dlsym(handle_, "f0r_get_plugin_info");
  get_param_info = (f0r_get_param_info_t)dlsym(handle_, "f0r_get_param_info");
  construct = (f0r_construct_t)dlsym(handle_, "f0r_construct");
  destruct = (f0r_destruct_t)dlsym(handle_, "f0r_destruct");
  set_param_value = (f0r_set_param_value_t)dlsym(handle_, "f0r_set_param_value");
  get_param_value = (f0r_get_param_value_t)dlsym(handle_, "f0r_get_param_value");
  update1 = (f0r_update_t)dlsym(handle_, "f0r_update");
  update2 = (f0r_update2_t)dlsym(handle_, "f0r_update2");

  get_plugin_info(&fi_);
  print();

  return true;
}

Frei0rImage::~Frei0rImage()
{
  if (instance_) {
    destruct(instance_);
  }
  if (handle_) {
    dlclose(handle_);
  }
}

void Frei0rImage::widthCallback(int width)
{
  unsigned int width_adjusted = (width / 8) * 8;
  setSize(width_adjusted, height_);
}

void Frei0rImage::heightCallback(int height)
{
  unsigned int height_adjusted = (height / 8) * 8;
  setSize(width_, height_adjusted);
}

void Frei0rImage::print()
{
  std::stringstream ss;
  ss << "{\n";
  ss << fi_.name << " by " << fi_.author << "\n";
  ss << "type: " << fi_.plugin_type << " " << plugin_types[fi_.plugin_type] << "\n";
  ss << "color model: " << fi_.color_model << "\n";
  ss << "frei0r_version: " << fi_.frei0r_version << "\n";
  ss << "major_version: " << fi_.major_version << "\n";
  ss << "minor_version: " << fi_.minor_version << "\n";
  ss << "num_params: " << fi_.num_params << "\n";
  for (int i = 0; i < fi_.num_params; ++i) {
    f0r_param_info_t info;
    get_param_info(&info, i);
    ss << "  " << i << " '" << info.name << "' " << param_types[info.type]
        << " '" << info.explanation << "'\n";
  }
  ss << "explanation: " << fi_.explanation << "\n";

  // for (size_t i = 0; i < 8 && i < out_frame_.size(); ++i) {
  //   ss << std::hex << out_frame_[i] << " ";
  // }
  ss << "\n";
  ss << "}\n";
  ROS_INFO_STREAM(ss.str());
}

bool Frei0rImage::setSize(unsigned int& width, unsigned int& height)
{
  ROS_INFO_STREAM(width << " x " << height);
  width = (width / 8) * 8;
  if (width == 0) {
    return false;
  }
  height = (height / 8) * 8;
  if (height == 0) {
    return false;
  }
  if ((width != width_) || (height != height_)) {
    width_ = width;
    height_ = height;
    if (instance_) {
      destruct(instance_);
    }
    instance_ = construct(width, height);
    const size_t num = width * height;  // * 4;
    in_frame_.resize(num);
    // out_frame_.resize(num);
  }
  return true;
}

void Frei0rImage::update(const ros::TimerEvent& event)
{
  if (!handle_) {
    return;
  }
  if (!instance_) {
    ROS_ERROR_STREAM("can't update without instance");
    return;
  }
  const double time_val = event.current_real.toSec();

  sensor_msgs::Image msg;
  msg.header.stamp = event.current_real;
  msg.encoding = "bgra8";
  msg.data.resize(width_ * height_ * 4);
  msg.width = width_;
  msg.height = height_;
  msg.step = width_ * 4;

  if ((fi_.plugin_type != F0R_PLUGIN_TYPE_MIXER2) &&
      (fi_.plugin_type != F0R_PLUGIN_TYPE_MIXER3)) {
    update1(instance_, time_val,
        &in_frame_[0],
        (uint32_t*)&msg.data[0]);
        // &out_frame_[0]);
  }

  pub_.publish(msg);
}

}  // namespace frei0r_image

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(frei0r_image::Frei0rImage, nodelet::Nodelet)
