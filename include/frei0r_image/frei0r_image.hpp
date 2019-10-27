/**
 * Copyright (c) 2019 Lucas Walter
 * Load frei0r plugins following these guidelines:
 * https://frei0r.dyne.org/codedoc/html/group__pluglocations.html
 */

#ifndef FREI0R_IMAGE_FREI0R_IMAGE_HPP
#define FREI0R_IMAGE_FREI0R_IMAGE_HPP

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
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

namespace frei0r_image
{

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

struct Instance
{
  Instance(unsigned int& width, unsigned int& height,
    f0r_construct_t construct,
    f0r_destruct_t destruct,
    f0r_update_t update1,
    f0r_update2_t update2,
    f0r_plugin_info fi,
    f0r_get_param_info_t get_param_info,
    f0r_get_param_value_t get_param_value,
    f0r_set_param_value_t set_param_value);

  ~Instance();
  f0r_instance_t instance_ = nullptr;

  void updateParams();

  void setParamValue(double value, const int ind)
  {
    set_param_value(instance_, reinterpret_cast<f0r_param_t>(&value), ind);
  }

  void setParamValue(const bool pre_value, const int ind)
  {
    double value = pre_value ? 1.0 : 0.0;
    set_param_value(instance_, reinterpret_cast<f0r_param_t>(&value), ind);
  }

  f0r_construct_t construct = nullptr;
  f0r_destruct_t destruct = nullptr;
  f0r_update_t update1 = nullptr;
  f0r_update2_t update2 = nullptr;
  f0r_get_param_info_t get_param_info = nullptr;
  f0r_get_param_value_t get_param_value = nullptr;
  f0r_set_param_value_t set_param_value = nullptr;

  std::map<int, bool> update_bools_;
  std::map<int, double> update_doubles_;
  void getValues();
  f0r_plugin_info fi_;

  void update(const ros::Time stamp);
  // TODO(lucasw) could be cv::Mat
  std::vector<uint32_t> in_frame_;
  // TODO(lucasw) needs to be ptr
  sensor_msgs::Image msg_;

  unsigned int width_ = 0;
  unsigned int height_ = 0;
};

struct Plugin
{
  Plugin(const std::string& plugin_name);
  ~Plugin();
  void print();
  f0r_init_t init;
  f0r_deinit_t deinit;

  f0r_construct_t construct = nullptr;
  f0r_destruct_t destruct = nullptr;
  f0r_get_param_info_t get_param_info = nullptr;
  f0r_get_param_value_t get_param_value = nullptr;
  f0r_set_param_value_t set_param_value = nullptr;

  void makeInstance(unsigned int width, unsigned int height)
  {
    instance_ = std::make_unique<Instance>(width, height,
        construct, destruct, update1, update2,
        fi_, get_param_info, get_param_value, set_param_value);
  }

  std::string plugin_name_;
  f0r_plugin_info fi_;
  f0r_get_plugin_info_t get_plugin_info = nullptr;

  f0r_update_t update1;
  f0r_update2_t update2;

  std::unique_ptr<Instance> instance_;
  void* handle_ = nullptr;

  const std::array<std::string, 4> plugin_types = {
      "filter", "source", "mixer2", "mixer3"};
  const std::array<std::string, 5> param_types = {
      "bool", "double", "color", "position", "string"};
};

class Frei0rImage : public nodelet::Nodelet
{
public:
  Frei0rImage();
  virtual void onInit();
  void widthCallback(int width);
  void heightCallback(int height);
  void boolCallback(bool value, int param_ind);
  void doubleCallback(double value, int param_ind);

  void selectPlugin(std::string plugin_name);

  void update(const ros::TimerEvent& event);
private:
  std::string new_plugin_name_ = "none";
  ros::Publisher pub_;
  ros::Timer timer_;
  std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;
  std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> select_plugin_ddr_;

  void setupPlugin();
  // std::map<int, std::map<std::string, std::shared_ptr<Frei0rImage>>> plugins_;

  unsigned int new_width_ = 0;
  unsigned int new_height_ = 0;

  std::unique_ptr<Plugin> plugin_;
};

}  // namespace frei0r_image

#endif  // FREI0R_IMAGE_FREI0R_IMAGE_HPP
