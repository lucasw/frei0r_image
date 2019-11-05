/**
 * Copyright 2019 Lucas Walter
 */

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <experimental/filesystem>
#include <dlfcn.h>
#include <frei0r.h>
#include <frei0r_image/LoadPlugin.h>
#include <frei0r_image/frei0r_image.hpp>
#include <map>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace frei0r_image
{

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
  plugin_name = sanitize(info.name);
  plugin_type = info.plugin_type;
  dlclose(handle);
  return true;
}

struct Selector
{
  std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::ServiceClient load_client_;
  const std::map<int, std::string> f0r_types_ = {
    {F0R_PLUGIN_TYPE_FILTER, "filter"},
    {F0R_PLUGIN_TYPE_SOURCE, "source"},
    {F0R_PLUGIN_TYPE_MIXER2, "mixer2"},
    {F0R_PLUGIN_TYPE_MIXER3, "mixer3"}
  };
  std::map<int, std::map<std::string, std::string>> enum_map_;

  Selector() :
    private_nh_("~")
  {
    load_client_ = nh_.serviceClient<LoadPlugin>("load_plugin");

    for (const auto& pair : f0r_types_) {
      enum_map_[pair.first]["none"] = "none";
    }

    bool load_from_path = true;
    private_nh_.getParam("load_from_path", load_from_path);
    if (load_from_path) {
      getPluginsFromDir();
    } else {
      getPluginsFromParam();
    }

    ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(private_nh_);
    for (const auto& pair : enum_map_) {
      const std::string frei0r_type_name = f0r_types_.at(pair.first);
      ddr_->registerEnumVariable<std::string>(frei0r_type_name, "none",
          boost::bind(&Selector::select, this, _1), frei0r_type_name, pair.second);
    }
    ddr_->publishServicesTopics();
  }

  bool getPluginsFromDir()
  {
    // TODO(lucasw) one of the ddr variables could be a path name
    std::vector<std::string> plugin_dirs = {
      // "/usr/lib/frei0r-1/",  // ubuntu frei0r-plugins puts them here
      // "/usr/local/lib/frei0r-1/",
      //  "/.frei0r-1/lib"  // TODO(lucasw) need to prefix $HOME to this
    };
    std::string custom_path = "/usr/lib/frei0r-1/";
    private_nh_.getParam("path", custom_path);
    ROS_INFO_STREAM("custom search path: " << custom_path);
    plugin_dirs.push_back(custom_path);

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
#if 0
          if (!((plugin_type == F0R_PLUGIN_TYPE_SOURCE) ||
                (plugin_type == F0R_PLUGIN_TYPE_FILTER))) {
            continue;
          }
#endif
          ROS_INFO_STREAM(plugin_type << " " << name);
          // const std::string name = info.name;
          enum_map_[plugin_type][name] = path;
        }
      } catch (std::experimental::filesystem::v1::__cxx11::filesystem_error& ex) {
        std::cout << dir << " " << ex.what() << "\n";
      }
    }

    for (const auto& pair : enum_map_) {
      private_nh_.setParam(f0r_types_.at(pair.first), pair.second);
    }
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

    return true;
  }

  bool getPluginsFromParam()
  {
    for (const auto& pair : enum_map_) {
      std::map<std::string, std::string> frei0r_locations;
      const std::string name = f0r_types_.at(pair.first);
      private_nh_.getParam(name, frei0r_locations);
      for (const auto& pair2 : frei0r_locations) {
        enum_map_[pair.first][pair2.first] = pair2.second;
      }
    }
    return true;
  }

  void select(const std::string plugin_path)
  {
    LoadPlugin srv;
    srv.request.plugin_path = plugin_path;
    if (!load_client_.waitForExistence(ros::Duration(0.1))) {
    }
    load_client_.call(srv);
  }
};

}  // namespace frei0r_image

int main(int argn, char* argv[])
{
  ros::init(argn, argv, "select_plugin");
  auto selector = frei0r_image::Selector();
  ros::spin();

  return 0;
}
