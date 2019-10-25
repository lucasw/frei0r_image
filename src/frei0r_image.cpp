/**
 * Copyright (c) 2019 Lucas Walter
 * Load frei0r plugins following these guidelines:
 * https://frei0r.dyne.org/codedoc/html/group__pluglocations.html
 */

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <experimental/filesystem>
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

std::string sanitize(const std::string& text)
{
  std::string text2 = text;
  // spaces aren't allowed
  for (char& c : text2) {
    if (c == ' ') {
      c = '_';
    }
  }

  text2 = "p" + text2;
  return text2;
}

void Frei0rImage::onInit()
{
  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image", 3);

  std::string path = "/usr/lib/frei0r-1/";
  // std::string name = "/usr/lib/frei0r-1/nois0r.so";
  plugin_name_ = path + "plasma.so";
  getPrivateNodeHandle().getParam("library", plugin_name_);
  loadLibrary(plugin_name_);

  new_width_ = 128;
  new_height_ = 128;
  setSize(new_width_, new_height_);

  ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(getPrivateNodeHandle());
  ddr_->registerVariable<int>("width", 8,
      boost::bind(&Frei0rImage::widthCallback, this, _1), "width", 8, 2048);
  ddr_->registerVariable<int>("height", 8,
      boost::bind(&Frei0rImage::heightCallback, this, _1), "height", 8, 2048);

  for (int i = 0; i < fi_.num_params; ++i) {
    // TODO(lucasw) create a control for each parameter
    f0r_param_info_t info;
    get_param_info(&info, i);
    // ss << "  " << i << " '" << info.name << "' " << param_types[info.type]
    //     << " '" << info.explanation << "'\n";
    const std::string param_name = sanitize(info.name);
    switch (info.type) {
      case (F0R_PARAM_BOOL): {
        ddr_->registerVariable<bool>(param_name, true,
            boost::bind(&Frei0rImage::boolCallback, this, _1, i),
        // ddr_->registerVariable<double>(param_name, true,
        //     boost::bind(&Frei0rImage::doubleCallback, this, _1, i),
            info.explanation);
      }
      case (F0R_PARAM_DOUBLE): {
        // starting with numbers isn't allowed, so prefix everything
        ROS_INFO_STREAM("'" << param_name << "'");
        ddr_->registerVariable<double>(param_name, 0.5,
            boost::bind(&Frei0rImage::doubleCallback, this, _1, i),
            info.explanation, 0.0, 1.0);
      }
      case (F0R_PARAM_COLOR): {
      }
      case (F0R_PARAM_POSITION): {
      }
      case (F0R_PARAM_STRING): {
      }
    }
  }

  ddr_->publishServicesTopics();

  timer_ = getPrivateNodeHandle().createTimer(ros::Duration(0.1),
      &Frei0rImage::update, this);
}

bool Frei0rImage::loadLibrary(const std::string& name)
{
  plugin_name_ = name;
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

  if (init == 0 || deinit == 0 || get_plugin_info == 0 ||
    get_param_info == 0 || construct == 0 || destruct == 0 ||
    set_param_value == 0 || get_param_value == 0 ||
    (update1 == 0 && update2 == 0)) {
    throw std::runtime_error("some symbols are missing in frei0r plugin");
  }

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
  new_width_ = (width / 8) * 8;
}

void Frei0rImage::heightCallback(int height)
{
  new_height_ = (height / 8) * 8;
}

void Frei0rImage::boolCallback(bool value, int param_ind)
{
  update_bools_[param_ind] = value;
}

void Frei0rImage::doubleCallback(double value, int param_ind)
{
  update_doubles_[param_ind] = value;
}

void Frei0rImage::print()
{
  std::stringstream ss;
  ss << "'" << plugin_name_ << "' {\n";
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
  // ROS_INFO_STREAM_THROTTLE(30, width << " x " << height);
  width = (width / 8) * 8;
  if (width == 0) {
    return false;
  }
  height = (height / 8) * 8;
  if (height == 0) {
    return false;
  }
  if ((width != width_) || (height != height_)) {
    getValues();
    width_ = width;
    height_ = height;
    if (instance_) {
      destruct(instance_);
    }
    instance_ = construct(width, height);
    msg_.data.resize(width_ * height_ * 4);
    msg_.encoding = "rgba8";
    msg_.width = width_;
    msg_.height = height_;
    msg_.step = width_ * 4;
    if (fi_.plugin_type == F0R_PLUGIN_TYPE_SOURCE) {
      return true;
    }
    const size_t num = width * height;  // * 4;
    in_frame_.resize(num);
    // out_frame_.resize(num);
  }
  return true;
}

void Frei0rImage::getValues()
{
  if (!instance_) {
    return;
  }
  for (int i = 0; i < fi_.num_params; ++i) {
    // TODO(lucasw) create a control for each parameter
    f0r_param_info_t info;
    get_param_info(&info, i);
    // ss << "  " << i << " '" << info.name << "' " << param_types[info.type]
    //     << " '" << info.explanation << "'\n";
    switch (info.type) {
      case (F0R_PARAM_BOOL): {
        double value;
        get_param_value(instance_, reinterpret_cast<void*>(&value), i);
        update_bools_[i] = value > 0.5;
      }
      case (F0R_PARAM_DOUBLE): {
        double value;
        get_param_value(instance_, reinterpret_cast<void*>(&value), i);
        update_doubles_[i] = value;
        // ROS_INFO_STREAM("'" << info.name << "': " << value);
      }
      case (F0R_PARAM_COLOR): {
      }
      case (F0R_PARAM_POSITION): {
      }
      case (F0R_PARAM_STRING): {
      }
    }
  }

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

  setSize(new_width_, new_height_);

  for (auto& pair : update_bools_) {
    double value = pair.second ? 1.0 : 0.0;
    set_param_value(instance_, reinterpret_cast<f0r_param_t>(&value),
        pair.first);
  }
  update_bools_.clear();

  for (auto& pair : update_doubles_) {
    set_param_value(instance_, reinterpret_cast<f0r_param_t>(&pair.second),
        pair.first);
  }
  update_doubles_.clear();

  const double time_val = event.current_real.toSec();

  msg_.header.stamp = event.current_real;

//  if ((fi_.plugin_type != F0R_PLUGIN_TYPE_MIXER2) &&
//      (fi_.plugin_type != F0R_PLUGIN_TYPE_MIXER3)) {
  if (fi_.plugin_type == F0R_PLUGIN_TYPE_SOURCE) {
    update1(instance_, time_val,
        nullptr,
        reinterpret_cast<uint32_t*>(&msg_.data[0]));
  }

  pub_.publish(msg_);
}

}  // namespace frei0r_image

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(frei0r_image::Frei0rImage, nodelet::Nodelet)
