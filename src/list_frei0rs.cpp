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
#include <iostream>
#include <map>
#include <memory>
#include <nodelet/nodelet.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>


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
        ROS_INFO_STREAM(entry.path());
      }
    } catch (std::experimental::filesystem::v1::__cxx11::filesystem_error& ex) {
      std::cout << dir << " " << ex.what() << "\n";
    }
  }

  if (plugin_names.size() == 0) {
    return 1;
  }

  std::map<int, std::map<std::string, std::shared_ptr<frei0r_image::Frei0rImage>>> plugins;

  std::map<std::string, bool> bad_frei0rs;
  bad_frei0rs["/usr/lib/frei0r-1/curves.so"] = true;

  for (const auto& plugin_name : plugin_names) {
    if (bad_frei0rs.count(plugin_name) > 0) {
      std::cout << "skipping " << plugin_name << "\n";
      continue;
    }
    auto plugin = std::make_shared<frei0r_image::Frei0rImage>();
    // Temp disable TODO(lucasw) fix this
#if 0
    plugin->loadLibrary(plugin_name);
    const int plugin_type = plugin->fi_.plugin_type;
    plugins[plugin_type][plugin_name] = plugin;
    std::cout << plugin_type << " " << plugin_name << "\n";
#endif
  }

#if 0
  for (const auto& plugin_pair : plugins[1]) {
    plugin_pair.second->print();
  }
#endif

  std::cout << std::endl;
  return 0;
}

