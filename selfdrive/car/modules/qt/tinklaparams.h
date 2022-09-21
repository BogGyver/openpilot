#pragma once
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;
#include <fstream>
using std::ifstream;
using std::ofstream;
#include <sstream>
#include <cstdlib> 
#include "selfdrive/common/params.h"

//const std::string tinkla_params_path = "/data/params";

bool tinkla_get_bool_param(const std::string &tinkla_param) {
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

void tinkla_set_bool_param(const std::string &tinkla_param,int tinkla_param_value) {
      ofstream ofile;
      ofile.open(tinkla_params_path + "/" + tinkla_param);
      if (ofile) {
        ofile << tinkla_param_value;
        ofile.close();
      }
}

float tinkla_get_float_param(const std::string &tinkla_param, float default_value) {
    ifstream ifile;
    ifile.open(tinkla_params_path + "/" + tinkla_param);
    if (!ifile) {
      //no file assume default_value and create
      ofstream ofile;
      ofile.open(tinkla_params_path + "/" + tinkla_param);
      if (ofile) {
        ofile << default_value;
        ofile.close();
      }
      return default_value;
    } else {
      float value;
      ifile >> value;
      ifile.close();
      return value;
    }
}

void tinkla_set_float_param(const std::string &tinkla_param,float tinkla_param_value) {
      ofstream ofile;
      ofile.open(tinkla_params_path + "/" + tinkla_param);
      if (ofile) {
        ofile << tinkla_param_value;
        ofile.close();
      }
}

std::string tinkla_get_str_param(const std::string &tinkla_param, std::string default_value) {
    ifstream ifile;
    ifile.open(tinkla_params_path + "/" + tinkla_param);
    if (!ifile) {
      //no file assume default_value and create
      ofstream ofile;
      ofile.open(tinkla_params_path + "/" + tinkla_param);
      if (ofile) {
        ofile << default_value;
        ofile.close();
      }
      return default_value;
    } else {
      std::stringstream strStream;
      strStream << ifile.rdbuf();
      std::string str = strStream.str();
      return str;
    }
}

void tinkla_set_str_param(const std::string &tinkla_param,std::string tinkla_param_value) {
      ofstream ofile;
      ofile.open(tinkla_params_path + "/" + tinkla_param);
      if (ofile) {
        ofile << tinkla_param_value;
        ofile.close();
      }
}
