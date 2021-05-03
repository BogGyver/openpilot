#pragma once

#include <stddef.h>
#include <map>
#include <string>
#include <sstream>
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;
#include <fstream>
using std::ifstream;
using std::ofstream;
#include <cstdlib>

#define ERR_NO_VALUE -33
const std::string tinkla_params_path = "/data/params";

class Params {
private:
  std::string params_path;

public:
  Params(bool persistent_param = false);
  Params(const std::string &path);

  // Delete a value
  int remove(const char *key);
  inline int remove(const std::string &key) {
    return remove (key.c_str());
  }

  // read all values
  int read_db_all(std::map<std::string, std::string> *params);

  // read a value
  std::string get(const char *key, bool block = false);

  inline std::string get(const std::string &key, bool block = false) {
    return get(key.c_str(), block);
  }

  template <class T>
  std::optional<T> get(const char *key, bool block = false) {
    std::istringstream iss(get(key, block));
    T value{};
    iss >> value;
    return iss.fail() ? std::nullopt : std::optional(value);
  }

  inline bool getBool(const std::string &key) {
    return getBool(key.c_str());
  }

  inline bool getBool(const char *key) {
    return get(key) == "1";
  }

  inline bool tinkla_get_bool_param(const std::string &tinkla_param) {
    ifstream ifile;
    ifile.open(tinkla_params_path + "/" + tinkla_param);
    if (!ifile) {
      //no file assume false and create
      ofstream ofile;
      ofile.open(tinkla_params_path + "/" + tinkla_param);
      if (ofile) {
        ofile << 0;
        ofile.close();
      }
      return false;
    } else {
      int value;
      ifile >> value;
      ifile.close();
      if (value == 0) {
        return false;
      } else {
        return true;
      }
    }
  }

  inline void tinkla_set_bool_param(const std::string &tinkla_param,int tinkla_param_value) {
      ofstream ofile;
      ofile.open(tinkla_params_path + "/" + tinkla_param);
      if (ofile) {
        ofile << tinkla_param_value;
        ofile.close();
      }
  }

  // write a value
  int put(const char* key, const char* val, size_t value_size);

  inline int put(const std::string &key, const std::string &val) {
    return put(key.c_str(), val.data(), val.size());
  }

  inline int putBool(const char *key, bool val) {
    return put(key, val ? "1" : "0", 1);
  }

  inline int putBool(const std::string &key, bool val) {
    return putBool(key.c_str(), val);
  }
};
