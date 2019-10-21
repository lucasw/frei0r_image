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
#include <vector>
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

typedef void (*f0r_update_t)(f0r_instance_t isntance, double time,
           const uint32_t* inframe, uint32_t* outframe);

typedef void (*f0r_update2_t)(f0r_instance_t isntance, double time,
      const uint32_t* inframe1,
      const uint32_t* inframe2,
      const uint32_t* inframe3,
      uint32_t* outframe);

int main(int argc, char** argv)
{
  std::vector<std::string> plugin_dirs = {
    "/usr/lib/frei0r-1/",  // ubuntu frei0r-plugins puts them here
    "/usr/local/lib/frei0r-1/",
    "/.frei0r-1/lib"  // TODO(lucasw) need to prefix $HOME to this
  };

  std::vector<std::string> plugins;
  for (const auto& dir : plugin_dirs) {
    if (!std::experimental::filesystem::exists(dir)) {
      continue;
    }
    try {
      for (const auto& entry : std::experimental::filesystem::directory_iterator(dir)) {
        plugins.push_back(entry.path());
        std::cout << entry.path() << "\n";
      }
    } catch (std::experimental::filesystem::v1::__cxx11::filesystem_error& ex) {
      std::cout << dir << " " << ex.what() << "\n";
    }
  }


  if (plugins.size() == 0) {
    return 1;
  }

  for (const auto& plugin : plugins) {
    // dl loader - TODO(lucasw) Replace with raii data structure
    void* handle = dlopen(plugin.c_str(), RTLD_NOW);
    std::cout << handle << " ";
    f0r_init_t init = (f0r_init_t)dlsym(handle, "f0r_init");
    f0r_get_plugin_info_t info = (f0r_get_plugin_info_t)dlsym(handle, "f0r_get_plugin_info");
    f0r_plugin_info fi;
    info(&fi);
    std::cout << fi.name << " by " << fi.author << "\n";
  }

  std::cout << std::endl;
  return 0;
}
