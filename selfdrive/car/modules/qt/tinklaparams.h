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

bool tinkla_get_bool_param(const std::string &tinkla_param);

void tinkla_set_bool_param(const std::string &tinkla_param,int tinkla_param_value);

float tinkla_get_float_param(const std::string &tinkla_param, float default_value);

void tinkla_set_float_param(const std::string &tinkla_param,float tinkla_param_value) ;

std::string tinkla_get_str_param(const std::string &tinkla_param, std::string default_value) ;

void tinkla_set_str_param(const std::string &tinkla_param,std::string tinkla_param_value);
