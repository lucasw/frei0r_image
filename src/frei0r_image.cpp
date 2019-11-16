/**
 * Copyright (c) 2019 Lucas Walter
 * Load frei0r plugins following these guidelines:
 * https://frei0r.dyne.org/codedoc/html/group__pluglocations.html
 */

#include <algorithm>
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
    if (!std::isalnum(c)) {
      c = '_';
    }
  }

  text2 = "p" + text2;
  return text2;
}

void Frei0rImage::onInit()
{
  pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image_out", 3);

#if 0
  std::map<std::string, bool> bad_frei0rs;
  bad_frei0rs["/usr/lib/frei0r-1/curves.so"] = true;

  for (const auto& plugin_name : plugin_names) {
    if (bad_frei0rs.count(plugin_name) > 0) {
      std::cout << "skipping " << plugin_name << "\n";
      continue;
    }
    auto plugin = std::make_shared<frei0r_image::Frei0rImage>();
    plugin->loadLibrary(plugin_name);
    const int plugin_type = plugin->fi_.plugin_type;
    plugins[plugin_type][plugin_name] = plugin;
    std::cout << plugin_type << " " << plugin_name << "\n";
  }

  for (const auto& plugin_pair : plugins[1]) {
    plugin_pair.second->print();
  }
#endif

  setupPlugin("none");
  load_plugin_srv_ = getPrivateNodeHandle().advertiseService("load_plugin",
      &Frei0rImage::loadPlugin, this);
  timer_ = getPrivateNodeHandle().createTimer(ros::Duration(0.1),
      &Frei0rImage::update, this);

  sub_[0] = getNodeHandle().subscribe<sensor_msgs::Image>("image_in0", 2,
      boost::bind(&Frei0rImage::imageCallback, this, _1, 0));
  sub_[1] = getNodeHandle().subscribe<sensor_msgs::Image>("image_in1", 2,
      boost::bind(&Frei0rImage::imageCallback, this, _1, 1));
  sub_[2] = getNodeHandle().subscribe<sensor_msgs::Image>("image_in2", 2,
      boost::bind(&Frei0rImage::imageCallback, this, _1, 2));
}

void Frei0rImage::imageCallback(const sensor_msgs::ImageConstPtr& msg, const size_t index)
{
  if ((!plugin_) || (!plugin_->instance_)) {
    return;
  }
  // TODO(lucasw) better off using cv bridge and converting to right
  // encoding.
  // TODO(lucasw) need to adjustWidth on these, and then resize the data
  // new_width_ = msg->width;
  // new_height_ = msg->height;
  // adjustWidthHeight(new_width_, new_height_);

#if 0
  if ((new_width_ == msg->width) && (new_height_ == msg->height)) {
    plugin_->instance_->image_in_msg_ = msg;
  } else {
    plugin_->instance_->image_in_msg_ = boost::make_shared<sensor_msgs::Image>();
    plugin_->instance_->image_in_msg_->encoding = msg->encoding;
    plugin_->instance_->image_in_msg_->width = new_width_;
    plugin_->instance_->image_in_msg_->height = new_height_;
    plugin_->instance_->image_in_msg_->step = msg->width * 4;
    plugin_->instance_->image_in_msg_->data.resize(new_width_ * new_height_);
    if (new_width_ == msg->width) {
      std::copy(msg->data.begin(), msg->data.begin() + new_width_ * new_height_ * 4,
          plugin_->instance_->image_in_msg_->data.begin());
    } else if ((msg->width > 8) && (msg->height > 8)) {
      for (size_t i = 0; i < new_height_; ++i) {
        std::copy(msg->data.begin() + i * (msg->width * 4),
                  msg->data.begin() + i * (msg->width * 4) + new_width_,
                  plugin_->instance_->image_in_msg_->data.begin() + i * (new_width_ * 4));
      }
    } else {
      // make a black background and copy the image into it
    }
  }
#endif

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgra8");
  } catch (cv_bridge::Exception& ex) {
    ROS_ERROR_THROTTLE(1.0, "cv bridge exception %s", ex.what());
    return;
  }
  cv::resize(cv_ptr->image, plugin_->instance_->image_in_[index],
      cv::Size(new_width_, new_height_), cv::INTER_NEAREST);
}

bool Frei0rImage::loadPlugin(LoadPlugin::Request& req, LoadPlugin::Response& resp)
{
  resp.success = setupPlugin(req.plugin_path);
  if (resp.success) {
  }
  return true;
}

// TODO(lucasw) pass in string to store error messages
bool Frei0rImage::setupPlugin(const std::string& plugin_name)
{
  if (plugin_name == "none") {
    if (plugin_) {
      plugin_ = nullptr;
      ddr_ = nullptr;
    }
    if (!ddr_) {
      // make an empty ddr just to keep client happy (though it won't like
      // the interruption in service, if it notices).
      ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(getPrivateNodeHandle());
      ddr_->publishServicesTopics();
    }
    return true;
  }

  std::unique_ptr<Plugin> plugin;
  try {
    plugin = std::make_unique<Plugin>(plugin_name);
  } catch (std::runtime_error& ex) {
    ROS_ERROR_STREAM(ex.what() << " '" << plugin_name << "'");
    return false;
  }
  ROS_INFO_STREAM(plugin_name);
  if (!plugin) {
    ROS_ERROR_STREAM("no plugin: '" << plugin_name << "'");
    return false;
  }

  plugin->makeInstance(new_width_, new_height_);
  if (!plugin->instance_) {
    ROS_ERROR_STREAM("no instance for '" << plugin_name << "'");
    return false;
  }

  plugin_ = std::move(plugin);

  ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(getPrivateNodeHandle());
  ddr_->registerVariable<int>("width", 320,
      boost::bind(&Frei0rImage::widthCallback, this, _1), "width", 8, 2048);
  ddr_->registerVariable<int>("height", 240,
      boost::bind(&Frei0rImage::heightCallback, this, _1), "height", 8, 2048);

  for (int i = 0; i < plugin_->fi_.num_params; ++i) {
    // TODO(lucasw) create a control for each parameter
    f0r_param_info_t info;
    plugin_->instance_->get_param_info(&info, i);
    // ss << "  " << i << " '" << info.name << "' " << param_types[info.type]
    //     << " '" << info.explanation << "'\n";
    const std::string param_name = sanitize(info.name);
    switch (info.type) {
      case (F0R_PARAM_BOOL): {
        ROS_INFO_STREAM(i << " bool '" << param_name << "'");
        ddr_->registerVariable<bool>(param_name, true,
            boost::bind(&Frei0rImage::boolCallback, this, _1, i),
        // ddr_->registerVariable<double>(param_name, true,
        //     boost::bind(&Frei0rImage::doubleCallback, this, _1, i),
            info.explanation);
        break;
      }
      case (F0R_PARAM_DOUBLE): {
        // starting with numbers isn't allowed, so prefix everything
        ROS_INFO_STREAM(i << " double '" << param_name << "'");
        ddr_->registerVariable<double>(param_name, 0.5,
            boost::bind(&Frei0rImage::doubleCallback, this, _1, i),
            info.explanation, 0.0, 1.0);
        break;
      }
      case (F0R_PARAM_COLOR): {
        ROS_INFO_STREAM(i << " color '" << param_name << "'");
        ddr_->registerVariable<double>(param_name + "_r", 0.5,
            boost::bind(&Frei0rImage::colorRCallback, this, _1, i),
            info.explanation, 0.0, 1.0);
        ddr_->registerVariable<double>(param_name + "_g", 0.5,
            boost::bind(&Frei0rImage::colorGCallback, this, _1, i),
            info.explanation, 0.0, 1.0);
        ddr_->registerVariable<double>(param_name + "_b", 0.5,
            boost::bind(&Frei0rImage::colorBCallback, this, _1, i),
            info.explanation, 0.0, 1.0);
        break;
      }
      case (F0R_PARAM_POSITION): {
        ROS_INFO_STREAM(i << " position '" << param_name << "'");
        ddr_->registerVariable<double>(param_name + "_x", 0.5,
            boost::bind(&Frei0rImage::positionXCallback, this, _1, i),
            info.explanation, 0.0, 1.0);
        ddr_->registerVariable<double>(param_name + "_y", 0.5,
            boost::bind(&Frei0rImage::positionYCallback, this, _1, i),
            info.explanation, 0.0, 1.0);
        break;
      }
      case (F0R_PARAM_STRING): {
        ROS_INFO_STREAM(i << " string '" << param_name << "'");
        ddr_->registerVariable<std::string>(param_name, "",
            boost::bind(&Frei0rImage::stringCallback, this, _1, i),
            info.explanation);
        break;
      }
    }
  }

  ddr_->publishServicesTopics();
  return true;
}

Plugin::Plugin(const std::string& name)
{
  if (name == "none") {
    return;
  }
  ROS_INFO_STREAM("loading " << name);
  plugin_name_ = name;
  handle_ = dlopen(name.c_str(), RTLD_NOW);
  if (!handle_) {
    throw std::runtime_error("no plugin");
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
    // throw std::runtime_error("some symbols are missing in frei0r plugin");
    ROS_ERROR_STREAM("some symbols are missing in frei0r plugin");
    // TODO(lucasw) throw
    throw std::runtime_error("bad plugin");
  }

  init();

  ROS_INFO_STREAM("get info");
  get_plugin_info(&fi_);
  print();
}

Plugin::~Plugin()
{
  if (handle_) {
    ROS_INFO_STREAM("shutting down " << plugin_name_);
    instance_ = nullptr;
    deinit();
    dlclose(handle_);
  }
}

void Frei0rImage::widthCallback(int width)
{
  new_width_ = width;
}

void Frei0rImage::heightCallback(int height)
{
  new_height_ = height;
}

void Frei0rImage::boolCallback(bool value, int param_ind)
{
  if ((!plugin_) || (!plugin_->instance_)) {
    return;
  }
  plugin_->instance_->update_bools_[param_ind] = value;
}

void Frei0rImage::doubleCallback(double value, int param_ind)
{
  if ((!plugin_) || (!plugin_->instance_)) {
    return;
  }
  plugin_->instance_->update_doubles_[param_ind] = value;
}

void Frei0rImage::colorRCallback(double value, int param_ind)
{
  if ((!plugin_) || (!plugin_->instance_)) {
    return;
  }
  plugin_->instance_->update_color_r_[param_ind] = value;
}

void Frei0rImage::colorGCallback(double value, int param_ind)
{
  if ((!plugin_) || (!plugin_->instance_)) {
    return;
  }
  plugin_->instance_->update_color_g_[param_ind] = value;
}

void Frei0rImage::colorBCallback(double value, int param_ind)
{
  if ((!plugin_) || (!plugin_->instance_)) {
    return;
  }
  plugin_->instance_->update_color_b_[param_ind] = value;
}

void Frei0rImage::positionXCallback(double value, int param_ind)
{
  if ((!plugin_) || (!plugin_->instance_)) {
    return;
  }
  plugin_->instance_->update_position_x_[param_ind] = value;
}

void Frei0rImage::positionYCallback(double value, int param_ind)
{
  if ((!plugin_) || (!plugin_->instance_)) {
    return;
  }
  plugin_->instance_->update_position_y_[param_ind] = value;
}

void Frei0rImage::stringCallback(const std::string value, int param_ind)
{
  if ((!plugin_) || (!plugin_->instance_)) {
    return;
  }
  plugin_->instance_->update_string_[param_ind] = value;
}

void Plugin::print()
{
  std::stringstream ss;
  ss << "'" << plugin_name_ << "' {\n";
  ss << fi_.name << " by " << fi_.author << "\n";
  ss << "type: " << fi_.plugin_type << " " << plugin_types[fi_.plugin_type] << "\n";
  ss << "color model: " << fi_.color_model << " " << color_models[fi_.color_model] << "\n";
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

void adjustWidthHeight(unsigned int& width, unsigned int& height)
{
  const unsigned align = 8;
  width -= width % 8;
  if (width == 0) {
    width = align;
  }
  height -= height % 8;
  if (height == 0) {
    height = align;
  }
}

Instance::Instance(unsigned int& width, unsigned int& height,
  f0r_construct_t construct,
  f0r_destruct_t destruct,
  f0r_update_t update1,
  f0r_update2_t update2,
  f0r_plugin_info fi,
  f0r_get_param_info_t get_param_info,
  f0r_get_param_value_t get_param_value,
  f0r_set_param_value_t set_param_value) :
  construct(construct),
  destruct(destruct),
  fi_(fi),
  update1(update1),
  update2(update2),
  get_param_info(get_param_info),
  get_param_value(get_param_value),
  set_param_value(set_param_value)
{
  adjustWidthHeight(width, height);
  ROS_INFO_STREAM(width << " x " << height);

  {
    width_ = width;
    height_ = height;
    instance_ = construct(width_, height_);
    // getValues();
    if (fi_.plugin_type == F0R_PLUGIN_TYPE_SOURCE) {
      return;
    }
    const size_t num = width * height;  // * 4;
    in_frame_.resize(num);
    // out_frame_.resize(num);
  }
}

Instance::~Instance()
{
  destruct(instance_);
}

void Instance::getValues()
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
        ROS_INFO_STREAM("bool '" << info.name << "': " << value);
        break;
      }
      case (F0R_PARAM_DOUBLE): {
        double value;
        get_param_value(instance_, reinterpret_cast<void*>(&value), i);
        update_doubles_[i] = value;
        ROS_INFO_STREAM("double '" << info.name << "': " << value);
        break;
      }
      case (F0R_PARAM_COLOR): {
        break;
      }
      case (F0R_PARAM_POSITION): {
        f0r_param_position_t pos;
        get_param_value(instance_, reinterpret_cast<f0r_param_t>(&pos), i);
        ROS_INFO_STREAM("position '" << info.name << "': " << pos.x << " " << pos.y);
        break;
      }
      case (F0R_PARAM_STRING): {
        break;
      }
    }  // switch on param type
  }  // loop through params
}

void Frei0rImage::update(const ros::TimerEvent& event)
{
  adjustWidthHeight(new_width_, new_height_);
  if (!plugin_) {
    return;
  }
  if ((!plugin_->instance_) ||
      (new_width_ != plugin_->instance_->width_) ||
      (new_height_ != plugin_->instance_->height_)) {
    // TODO(lucasw) currently this will reset all parameter values,
    // need to copy them out to update_ maps.
    plugin_->makeInstance(new_width_, new_height_);
  }

  plugin_->instance_->updateParams();
  plugin_->instance_->update(event.current_real);

  if (plugin_->instance_->image_out_msg_) {
    pub_.publish(plugin_->instance_->image_out_msg_);
  }
}

void Instance::updateParams()
{
  for (auto& pair : update_bools_) {
    setParamValue(pair.second, pair.first);
  }
  update_bools_.clear();

  for (auto& pair : update_doubles_) {
    setParamValue(pair.second, pair.first);
  }
  update_doubles_.clear();

  // color
  for (auto& pair : update_color_r_) {
    setColorR(pair.second, pair.first);
  }
  update_color_r_.clear();

  for (auto& pair : update_color_g_) {
    setColorG(pair.second, pair.first);
  }
  update_color_g_.clear();

  for (auto& pair : update_color_b_) {
    setColorB(pair.second, pair.first);
  }
  update_color_b_.clear();

  // position
  for (auto& pair : update_position_x_) {
    setPositionX(pair.second, pair.first);
  }
  update_position_x_.clear();

  for (auto& pair : update_position_y_) {
    setPositionY(pair.second, pair.first);
  }
  update_position_y_.clear();

  for (auto& pair : update_string_) {
    setString(pair.second, pair.first);
  }
  update_string_.clear();
}

#if 0
Plugin::update(const ros::Time stamp)
{
  if (!instance_) {
    return;
  }
  instance_->update(stamp)
}
#endif

void Instance::update(const ros::Time stamp)
{
  const auto width = width_;
  const auto height = height_;
  if ((width < 8) || (height < 8)) {
    return;
  }
  image_out_msg_ = sensor_msgs::ImagePtr(new sensor_msgs::Image);
  image_out_msg_->header.stamp = stamp;
  image_out_msg_->data.resize(width * height * 4);
  image_out_msg_->encoding = "bgra8";
  image_out_msg_->width = width;
  image_out_msg_->height = height;
  image_out_msg_->step = width * 4;

  const double time_val = stamp.toSec();

  const auto image_out_data = reinterpret_cast<uint32_t*>(&image_out_msg_->data[0]);
  // if ((fi_.plugin_type != F0R_PLUGIN_TYPE_MIXER2) &&
  //     (fi_.plugin_type != F0R_PLUGIN_TYPE_MIXER3)) {
  switch (fi_.plugin_type) {
    case (F0R_PLUGIN_TYPE_FILTER): {
      if (!image_in_[0].empty()) {
        // TODO(lucasw) image_in_msg width and height may not be
        // multiples of 8
        {
          const int i = 0;
          cv::resize(image_in_[i], image_in_[i],
              cv::Size(width, height),
              cv::INTER_NEAREST);
        }
        update1(instance_, time_val,
            reinterpret_cast<uint32_t*>(&image_in_[0].data[0]),
            image_out_data);
      }
      break;
    }
    case  (F0R_PLUGIN_TYPE_SOURCE): {
      update1(instance_, time_val,
          nullptr,
          image_out_data);
      break;
    }
    case (F0R_PLUGIN_TYPE_MIXER2): {
      if (!image_in_[0].empty() && !image_in_[0].empty()) {
        for (size_t i = 0; i < 2; ++i) {
          cv::resize(image_in_[i], image_in_[i],
              cv::Size(width, height),
              cv::INTER_NEAREST);
        }
        update2(instance_, time_val,
            reinterpret_cast<uint32_t*>(&image_in_[0].data[0]),
            reinterpret_cast<uint32_t*>(&image_in_[1].data[0]),
            nullptr,
            image_out_data);
      }
      break;
    }
    case (F0R_PLUGIN_TYPE_MIXER3): {
      if (!image_in_[0].empty() && !image_in_[0].empty()) {
        for (size_t i = 0; i < 3; ++i) {
          cv::resize(image_in_[i], image_in_[i],
              cv::Size(width, height),
              cv::INTER_NEAREST);
        }
        update2(instance_, time_val,
            reinterpret_cast<uint32_t*>(&image_in_[0].data[0]),
            reinterpret_cast<uint32_t*>(&image_in_[1].data[0]),
            reinterpret_cast<uint32_t*>(&image_in_[2].data[0]),
            image_out_data);
      }
      break;
    }
  }
}

}  // namespace frei0r_image

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(frei0r_image::Frei0rImage, nodelet::Nodelet)
