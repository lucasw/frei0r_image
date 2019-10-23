/**
 * Copyright (c) 2019 Lucas Walter
 * Load frei0r plugins following these guidelines:
 * https://frei0r.dyne.org/codedoc/html/group__pluglocations.html
 */

#include <experimental/filesystem>
// TODO(lucasw) there is a C++ header in the latest frei0r sources,
// but it isn't in Ubuntu 18.04 released version currently
// #define _UNIX03_SOURCE
#include <dlfcn.h>
#include <frei0r.h>
#include <iostream>
#include <map>
#include <nodelet/nodelet.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>

typedef int  (*f0r_init_t)();
typedef void (*f0r_deinit_t)();
typedef void (*f0r_get_plugin_info_t)(f0r_plugin_info_t* info);
typedef void (*f0r_get_param_info_t)(f0r_param_info_t* info,
             int param_index);
typedef f0r_instance_t (*f0r_construct_t)(int width, int height);
typedef void (*f0r_destruct_t)(f0r_instance_t instance);

typedef void (*f0r_set_param_value_t)(f0r_instance_t instance,
            f0r_param_t param, int param_index);

typedef void (*f0r_get_param_value_t)(f0r_instance_t instance,
            f0r_param_t param, int param_index);

typedef void (*f0r_update_t)(f0r_instance_t instance, double time,
           const uint32_t* inframe, uint32_t* outframe);

typedef void (*f0r_update2_t)(f0r_instance_t instance, double time,
      const uint32_t* inframe1,
      const uint32_t* inframe2,
      const uint32_t* inframe3,
      uint32_t* outframe);

namespace frei0r_image
{

class Frei0rImage : public nodelet::Nodelet
{
public:
  Frei0rImage()
  {
  }

  virtual void onInit()
  {
    pub_ = getNodeHandle().advertise<sensor_msgs::Image>("image", 3);

    std::string name = "/usr/lib/frei0r-1/nois0r.so";
    getPrivateNodeHandle().getParam("library", name);

    handle_ = dlopen(name.c_str(), RTLD_NOW);
    if (!handle_) {
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

    get_plugin_info(&fi_);
    int width = 8;
    int height = 8;
    getPrivateNodeHandle().getParam("width", width);
    getPrivateNodeHandle().getParam("height", height);
    unsigned int width_adjusted = (width / 8) * 8;
    unsigned int height_adjusted = (height / 8) * 8;
    setSize(width_adjusted, height_adjusted);

    timer_ = getPrivateNodeHandle().createTimer(ros::Duration(0.1),
        &Frei0rImage::update, this);
  }

  ~Frei0rImage()
  {
    if (instance_) {
      destruct(instance_);
    }
    if (handle_) {
      dlclose(handle_);
    }
  }

  const std::array<std::string, 4> plugin_types = {
      "filter", "source", "mixer2", "mixer3"};
  const std::array<std::string, 5> param_types = {
      "bool", "double", "color", "position", "string"};

  void print()
  {
    std::cout << "{\n";
    std::cout << fi_.name << " by " << fi_.author << "\n";
    std::cout << "type: " << fi_.plugin_type << " " << plugin_types[fi_.plugin_type] << "\n";
    std::cout << "color model: " << fi_.color_model << "\n";
    std::cout << "frei0r_version: " << fi_.frei0r_version << "\n";
    std::cout << "major_version: " << fi_.major_version << "\n";
    std::cout << "minor_version: " << fi_.minor_version << "\n";
    std::cout << "num_params: " << fi_.num_params << "\n";
    for (int i = 0; i < fi_.num_params; ++i) {
      f0r_param_info_t info;
      get_param_info(&info, i);
      std::cout << "  " << i << " '" << info.name << "' " << param_types[info.type]
          << " '" << info.explanation << "'\n";
    }
    std::cout << "explanation: " << fi_.explanation << "\n";

    for (size_t i = 0; i < 8 && i < out_frame_.size(); ++i) {
      std::cout << std::hex << out_frame_[i] << " ";
    }
    std::cout << "\n";
    std::cout << "}\n";
  }

  bool setSize(unsigned int& width, unsigned int& height)
  {
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

  f0r_plugin_info fi_;
  f0r_get_plugin_info_t get_plugin_info;
  f0r_get_param_info_t get_param_info;
  f0r_get_param_value_t get_param_value;
  f0r_set_param_value_t set_param_value;

  void update(const ros::TimerEvent& event)
  {
    if (!handle_) {
      return;
    }
    if (!instance_) {
      std::cerr << "can't update without instance\n";
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
  f0r_update_t update1;
  f0r_update2_t update2;
private:
  ros::Publisher pub_;
  ros::Timer timer_;

  unsigned int width_ = 0;
  unsigned int height_ = 0;

  f0r_init_t init;
  f0r_deinit_t deinit;
  f0r_construct_t construct;
  f0r_destruct_t destruct;

  // TODO(lucasw) could be cv::Mat
  std::vector<uint32_t> in_frame_;
  std::vector<uint32_t> out_frame_;

  void* handle_ = nullptr;
  // TODO(lucasw) the instance and the plugin should be separate in general,
  // though here there will just be one plugin and one instance.
  f0r_instance_t instance_ = nullptr;
};

}  // namespace frei0r_image

#if 0
int main(int argc, char** argv)
{
  std::vector<std::string> plugin_dirs = {
    "/usr/lib/frei0r-1/",  // ubuntu frei0r-plugins puts them here
    "/usr/local/lib/frei0r-1/",
    "/.frei0r-1/lib"  // TODO(lucasw) need to prefix $HOME to this
  };

  std::vector<std::string> plugin_names;
  for (const auto& dir : plugin_dirs) {
    if (!std::experimental::filesystem::exists(dir)) {
      continue;
    }
    try {
      for (const auto& entry : std::experimental::filesystem::directory_iterator(dir)) {
        plugin_names.push_back(entry.path());
        // std::cout << entry.path() << "\n";
      }
    } catch (std::experimental::filesystem::v1::__cxx11::filesystem_error& ex) {
      std::cout << dir << " " << ex.what() << "\n";
    }
  }

  if (plugin_names.size() == 0) {
    return 1;
  }

  std::map<int, std::map<std::string, std::shared_ptr<Frei0rImage>>> plugins;

  // {
  //   const std::string plugin_name = "/usr/lib/frei0r-1/rgbnoise.so";
  for (const auto& plugin_name : plugin_names) {
    auto plugin = std::make_shared<Frei0rImage>(plugin_name);
    plugins[plugin->fi_.plugin_type][plugin_name] = plugin;
  }

  for (const auto& plugin_pair : plugins[1]) {
    plugin_pair.second->update();
    plugin_pair.second->print();
  }

  //plugins[1][

  std::cout << std::endl;
  return 0;
}
#endif

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(frei0r_image::Frei0rImage, nodelet::Nodelet)
