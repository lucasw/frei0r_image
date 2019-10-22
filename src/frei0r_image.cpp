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

typedef void (*f0r_update_t)(f0r_instance_t instance, double time,
           const uint32_t* inframe, uint32_t* outframe);

typedef void (*f0r_update2_t)(f0r_instance_t instance, double time,
      const uint32_t* inframe1,
      const uint32_t* inframe2,
      const uint32_t* inframe3,
      uint32_t* outframe);

struct Frei0rPlugin
{
  Frei0rPlugin(const std::string& name)
  {
    handle_ = dlopen(name.c_str(), RTLD_NOW);
    std::cout << name << " " << handle_ << "\n";

    init = (f0r_init_t)dlsym(handle_, "f0r_init");
    deinit = (f0r_deinit_t)dlsym(handle_, "f0r_deinit");
    get_plugin_info = (f0r_get_plugin_info_t)dlsym(handle_, "f0r_get_plugin_info");
    get_param_info = (f0r_get_param_info_t)dlsym(handle_, "f0r_get_param_info");
    construct = (f0r_construct_t)dlsym(handle_, "f0r_construct");
    destruct = (f0r_destruct_t)dlsym(handle_, "f0r_destruct");
    set_param_value = (f0r_set_param_value_t)dlsym(handle_, "f0r_set_param_value");
    get_param_value = (f0r_get_param_value_t)dlsym(handle_, "f0r_get_param_value");
    update = (f0r_update_t)dlsym(handle_, "f0r_update");
    update2 = (f0r_update2_t)dlsym(handle_, "f0r_update2");

    get_plugin_info(&fi_);
    print();
  }

  ~Frei0rPlugin()
  {
    if (handle_) {
      dlclose(handle_);
    }
  }

  void print()
  {
    std::cout << fi_.name << " by " << fi_.author << "\n";
    std::cout << "type: " << fi_.plugin_type << "\n";
    std::cout << "color model: " << fi_.color_model << "\n";
    std::cout << "frei0r_version: " << fi_.frei0r_version << "\n";
    std::cout << "major_version: " << fi_.major_version << "\n";
    std::cout << "minor_version: " << fi_.minor_version << "\n";
    std::cout << "num_params: " << fi_.num_params << "\n";
    std::cout << "explanation: " << fi_.explanation << "\n";
  }

  f0r_init_t init;
  f0r_deinit_t deinit;
  f0r_construct_t construct;
  f0r_destruct_t destruct;
  f0r_get_plugin_info_t get_plugin_info;
  f0r_plugin_info fi_;
  f0r_get_param_info_t get_param_info;
  f0r_get_param_value_t get_param_value;
  f0r_set_param_value_t set_param_value;
  f0r_update_t update;
  f0r_update2_t update2;

  void* handle_ = nullptr;
};

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

  std::map<std::string, std::shared_ptr<Frei0rPlugin>> plugins;

  // {
  //   const std::string plugin_name = "/usr/lib/frei0r-1/rgbnoise.so";
  for (const auto& plugin_name : plugin_names) {
    plugins[plugin_name] = std::make_shared<Frei0rPlugin>(plugin_name);
  }

  std::cout << std::endl;
  return 0;
}
