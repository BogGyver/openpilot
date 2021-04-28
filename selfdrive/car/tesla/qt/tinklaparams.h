#pragma once
#include <iostream>
using std::cerr;
using std::cout;
using std::endl;
#include <fstream>
using std::ifstream;
using std::ofstream;
#include <cstdlib> 

const std::string tinkla_params_path = "/data/params";

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
