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

  // TODO(lucasw) one of the ddr variables could be a path name
  std::vector<std::string> plugin_dirs = {
    // "/usr/lib/frei0r-1/",  // ubuntu frei0r-plugins puts them here
    // "/usr/local/lib/frei0r-1/",
     //  "/.frei0r-1/lib"  // TODO(lucasw) need to prefix $HOME to this
  };
  std::string custom_path = "/usr/lib/frei0r-1/";
  getPrivateNodeHandle().getParam("path", custom_path);
  plugin_dirs.push_back(custom_path);

  select_plugin_ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(getNodeHandle());

  std::map<std::string, std::string> enum_map;
  enum_map["none"] = "none";

  // std::vector<std::string> plugin_names;
  for (const auto& dir : plugin_dirs) {
    if (!std::experimental::filesystem::exists(dir)) {
      continue;
    }
    try {
      for (const auto& entry : std::experimental::filesystem::directory_iterator(dir)) {
        // TODO(lucasw) get the name and type of the plugin
        // and use it here.
        const auto path = entry.path();
        std::string name;
        int plugin_type = 0;
        // ROS_INFO_STREAM(path);
        const bool rv = getPluginInfo(path, name, plugin_type);
        if (!rv) {
          continue;
        }
        // if (plugin_type != F0R_PLUGIN_TYPE_SOURCE) {
        if (plugin_type != F0R_PLUGIN_TYPE_FILTER) {
          continue;
        }
        ROS_INFO_STREAM(plugin_type << " " << name);
        //const std::string name = info.name;
        enum_map[name] = path;
      }
    } catch (std::experimental::filesystem::v1::__cxx11::filesystem_error& ex) {
      std::cout << dir << " " << ex.what() << "\n";
    }
  }
  select_plugin_ddr_->registerEnumVariable<std::string>("frei0r", "none",
      boost::bind(&Frei0rImage::selectPlugin, this, _1), "frei0r", enum_map);

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

  select_plugin_ddr_->publishServicesTopics();

  timer_ = getPrivateNodeHandle().createTimer(ros::Duration(0.1),
      &Frei0rImage::update, this);

  sub_ = getNodeHandle().subscribe("image_in", 1, &Frei0rImage::imageCallback, this);
}

void Frei0rImage::imageCallback(const sensor_msgs::ImagePtr& msg)
{
  if ((!plugin_) || (!plugin_->instance_)) {
    return;
  }
  // TODO(lucasw) need to adjustWidth on these, and then resize the data
  new_width_ = msg->width;
  new_height_ = msg->height;
  plugin_->instance_->image_in_msg_ = msg;
}

void Frei0rImage::selectPlugin(std::string plugin_name)
{
  ROS_INFO_STREAM(plugin_name);
  new_plugin_name_ = plugin_name;
}

void Frei0rImage::setupPlugin()
{
  ROS_INFO_STREAM(new_plugin_name_);
  if (!plugin_) {
    ROS_ERROR_STREAM("no plugin");
    return;
  }
  if (!plugin_->instance_) {
    ROS_ERROR_STREAM("no instance");
    return;
  }
  ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(getPrivateNodeHandle());
  ddr_->registerVariable<int>("width", 8,
      boost::bind(&Frei0rImage::widthCallback, this, _1), "width", 8, 2048);
  ddr_->registerVariable<int>("height", 8,
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
        break;
      }
      case (F0R_PARAM_POSITION): {
        ROS_INFO_STREAM(i << " position '" << param_name << "'");
        ddr_->registerVariable<double>(param_name + "_x", 0.5,
            boost::bind(&Frei0rImage::colorXCallback, this, _1, i),
            info.explanation, 0.0, 1.0);
        ddr_->registerVariable<double>(param_name + "_y", 0.5,
            boost::bind(&Frei0rImage::colorYCallback, this, _1, i),
            info.explanation, 0.0, 1.0);
        break;
      }
      case (F0R_PARAM_STRING): {
        ROS_INFO_STREAM(i << " string '" << param_name << "'");
        break;
      }
    }
  }

  ddr_->publishServicesTopics();
}

bool getPluginInfo(const std::string& name, std::string& plugin_name, int& plugin_type)
{
  void* handle = dlopen(name.c_str(), RTLD_NOW);
  if (!handle) {
    return false;
  }
  f0r_get_plugin_info_t get_plugin_info = (f0r_get_plugin_info_t)dlsym(handle, "f0r_get_plugin_info");
  if (!get_plugin_info) {
    dlclose(handle);
    return false;
  }
  f0r_plugin_info_t info;
  get_plugin_info(&info);
  plugin_name = info.name;
  plugin_type = info.plugin_type;
  dlclose(handle);
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
    // TODO(lucasw) throw
    return;
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
    return;
  }

  init();

  ROS_INFO_STREAM("get info");
  get_plugin_info(&fi_);
  print();
}

Plugin::~Plugin()
{
  if (handle_) {
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

void Frei0rImage::colorXCallback(double value, int param_ind)
{
  if ((!plugin_) || (!plugin_->instance_)) {
    return;
  }
  plugin_->instance_->update_position_x_[param_ind] = value;
}

void Frei0rImage::colorYCallback(double value, int param_ind)
{
  if ((!plugin_) || (!plugin_->instance_)) {
    return;
  }
  plugin_->instance_->update_position_y_[param_ind] = value;
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
  width = (width / align) * align;
  if (width == 0) {
    width = align;
  }
  height = (height / align) * align;
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
    msg_.data.resize(width_ * height_ * 4);
    msg_.encoding = "bgra8";
    msg_.width = width_;
    msg_.height = height_;
    msg_.step = width_ * 4;
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

  if (new_plugin_name_ == "none") {
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
    return;
  }
  if ((!plugin_) || (new_plugin_name_ != plugin_->plugin_name_)) {
    if (plugin_) {
      ROS_INFO_STREAM(new_plugin_name_ << " " << plugin_->plugin_name_);
    }
    plugin_ = std::make_unique<Plugin>(new_plugin_name_);
    plugin_->makeInstance(new_width_, new_height_);
    setupPlugin();
  }
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

  pub_.publish(plugin_->instance_->msg_);
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

  for (auto& pair : update_position_x_) {
    setPositionX(pair.second, pair.first);
  }
  update_position_x_.clear();

  for (auto& pair : update_position_y_) {
    setPositionY(pair.second, pair.first);
  }
  update_position_y_.clear();
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
  msg_.header.stamp = stamp;

  const double time_val = stamp.toSec();

  // if ((fi_.plugin_type != F0R_PLUGIN_TYPE_MIXER2) &&
  //     (fi_.plugin_type != F0R_PLUGIN_TYPE_MIXER3)) {
  switch (fi_.plugin_type) {
    case (F0R_PLUGIN_TYPE_FILTER): {
      if (image_in_msg_) {
        // TODO(lucasw) image_in_msg width and height may not be
        // multiples of 8
        update1(instance_, time_val,
            reinterpret_cast<uint32_t*>(&image_in_msg_->data[0]),
            reinterpret_cast<uint32_t*>(&msg_.data[0]));
      }
      break;
    }
    case  (F0R_PLUGIN_TYPE_SOURCE): {
      update1(instance_, time_val,
          nullptr,
          reinterpret_cast<uint32_t*>(&msg_.data[0]));
      break;
    }
  }
}

}  // namespace frei0r_image

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(frei0r_image::Frei0rImage, nodelet::Nodelet)
