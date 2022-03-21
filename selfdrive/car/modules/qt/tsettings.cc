#include <string>
#include <iostream>
#include <sstream>
#include <cassert>

#include "tsettings.hpp"



#include "tcontrols.hpp"

#include "num_param.h"


TinklaTogglesPanel::TinklaTogglesPanel(SettingsWindow *parent) : ListWidget(parent) {

  std::vector<std::tuple<QString, QString, QString, QString>> tinkla_toggles{
  // param, title, desc, icon
    {"TinklaHso",
      "Enable HSO",
      "Enables Human Steering Override (HSO) module without disengaging OpenPilot.",
      "../assets/offroad/icon_warning.png",
      },
    {"TinklaHao",
      "Enable HAO",
      "Enables Human Accelerator Override (HSO) module without disengaging OpenPilot.",
      "../assets/offroad/icon_warning.png",
      },
    {"TinklaAlc",
      "Enable ALC",
      "Enables automatic lane change with just the tap of the turn signal stalk.  Your attention is required at all times to use this feature.",
      "../assets/offroad/icon_warning.png",
      },
    {"TinklaEnableOPLong",
      "Enable OP Long Control",
      "AP1/AP2: Enables OP Long Control and disables the AP ACC. Requires 2 pandas for MS AP2/MX AP1",
      "../assets/offroad/icon_warning.png",
      },
    {"TinklaTurnScreenOff",
      "Turn screen off while engaged",
      "Keeps device screen off even when engaged. It wakes the screen any time a message is shown.",
      "../assets/offroad/icon_settings.png",
      },
    {"TinklaDebugMode",
      "Show Debug View",
      "Shows the image captured by the road camera, including detected path and lanes.",
      "../assets/offroad/icon_settings.png",
      },
    {"TinklaHideGps",
      "Hide GPS Warnings",
      "Hides the GPS warning when user doesn't care about them.",
      "../assets/offroad/icon_settings.png",
      },
  };
  Params params;
  for (auto &[param, title, desc, icon] : tinkla_toggles) {
    auto toggle = new TinklaParamControl(param, title, desc, icon, this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    if (!locked) {
      connect(uiState(), &UIState::offroadTransition, toggle, &ParamControl::setEnabled);
    }
    addItem(toggle);
  };
  addItem(new NumParamControl("ALC delay", "The time, in seconds, that ALC will wait and keep the turn signal on and check blind spot monitoring (when available) before automatically starting the lange change.", 
      "ALC delay:", 
      "Enter time in seconds.", 
      "s",
      "TinklaAlcDelay", 2.0));
  addItem(new NumParamControl("HSO numb period", "The time, in seconds, to delay the reengagement of LKAS after HSO has been engaged by user by taking control of steering.", 
      "HSO numb period:", 
      "Enter time in seconds.", 
      "s",
      "TinklaHsoNumbPeriod", 1.5));
  QPushButton *flash_btn = new QPushButton("Flash Panda");
  flash_btn->setObjectName("flash_btn");
  
  QObject::connect(flash_btn, &QPushButton::clicked, [=](){
    QProcess::startDetached("/data/openpilot/panda/board/flashPanda");
  });

  QPushButton *flash_pedal_btn = new QPushButton("Flash Pedal");
  flash_pedal_btn->setObjectName("flash_pedal_btn");
  
  QObject::connect(flash_pedal_btn, &QPushButton::clicked, [=](){
    QProcess::startDetached("/data/openpilot/panda/board/pedal/flashPedal");
  });

  QPushButton *vin_radar_btn = new QPushButton("Radar VIN Learn");
  vin_radar_btn->setObjectName("vin_radar_btn");
  
  QObject::connect(vin_radar_btn, &QPushButton::clicked, [=](){
    QProcess::startDetached("/data/openpilot/selfdrive/car/modules/radarFlasher/flashTeslaRadar");
  });

  setStyleSheet(R"(
    #flash_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #flash_btn:pressed { background-color: #4a4a4a; }
    #flash_pedal_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #flash_pedal_btn:pressed { background-color: #4a4a4a; }
    #vin_radar_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #vin_radar_btn:pressed { background-color: #4a4a4a; }
  )");
  addItem(flash_btn);
  addItem(flash_pedal_btn);
  addItem(vin_radar_btn);
}

TeslaTogglesPanel::TeslaTogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  
  std::vector<std::tuple<QString, QString, QString, QString>> tinkla_toggles{
  // param, title, desc, icon
    {"TinklaPost1916Fix",
    "Tesla software post 2019.16",
    "Use the DBC for Tesla software after 2019.16 when some of the messages changed.",
    "../assets/offroad/icon_settings.png",
    },
    {"TinklaAllowHumanLong",
    "preAP: just control LKAS",
    "Allows the user to perform longitudinal control via accel/brake without disengaging OP. Requires reboot.",
    "../assets/offroad/icon_speed_limit.png",
    },
    {"TinklaEnablePedal",
    "preAP: Use pedal",
    "Enables the use of the Pedal Interceptor to control the speed of your pre-AutoPilot Tesla. Requires Pedal Interceptor hardware connected to CAN2. Requires reboot.",
    "../assets/offroad/icon_speed_limit.png",
    },
    {"TinklaPedalCanZero",
    "preAP: Pedal on CAN0",
    "Uses CAN0 for pedal interceptor. Default (and safest option) is CAN2. Only enable if you know what you're doing.",
    "../assets/offroad/icon_speed_limit.png",
    },
    {"TinklaAutoResumeACC",
    "preAP: AutoResume ACC",
    "Enables the use of the AutoResume mode ACC instead full disengagement. Works with both CC.",
    "../assets/offroad/icon_speed_limit.png",
    },
    {"TinklaHasIcIntegration",
    "preAP: Use Tinkla Buddy",
    "Enables IC integration via Tinkla Buddy. Only enable if you have a Tinkla Buddy licensed and installed.",
    "../assets/offroad/icon_speed_limit.png",
    },
    {"TinklaUseTeslaRadar",
    "preAP: Use Tesla Radar",
    "Enables the use of the Tesla Radar for pre-AutoPilot Tesla Model S. Requires Tesla Bosch radar hardware conencted to CAN1. Requires reboot.",
    "../assets/offroad/icon_settings.png",
    },
    {"TinklaTeslaRadarBehindNosecone",
    "preAP: Radar behind nosecone",
    "Enables the use of the Tesla Radar behind the nosecone for pre-AutoPilot Tesla Model S. Requires Tesla Bosch radar hardware conencted to CAN1. Requires reboot.",
    "../assets/offroad/icon_settings.png",
    },
    {"TinklaHasIBooster",
    "preAP: Car has iBooster",
    "Set to true if you retrofitted Tesla Model S iBooster on pre-AutoPilot cars. Requires reboot.",
    "../assets/offroad/icon_settings.png",
    },
    {"TinklaForceTeslaPreAP",
    "preAP: Force PreAP Tesla Model S ",
    "Forces the fingerprint to match a PreAP Tesla Model S. Requires reboot.",
    "../assets/offroad/icon_settings.png",
    },
  };
  Params params;
  for (auto &[param, title, desc, icon] : tinkla_toggles) {
    auto toggle = new TinklaParamControl(param, title, desc, icon, this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    if (!locked) {
      connect(uiState(), &UIState::offroadTransition, toggle, &ParamControl::setEnabled);
    }
    addItem(toggle);
  };
  addItem(new NumParamControl("Radar offset", "The distance, in meters from center of car, the radar is offset.",
  "Radar offset:",
  "Enter distance in meters.",
  "m",
  "TinklaRadarOffset", 0.0));
  QPushButton *flash_btn = new QPushButton("Flash EPAS");
  flash_btn->setObjectName("flash_btn");
  
  QObject::connect(flash_btn, &QPushButton::clicked, [=](){
    QProcess::startDetached("/data/openpilot/selfdrive/car/modules/teslaEpasFlasher/flashTeslaEPAS");
  });

  setStyleSheet(R"(
    #flash_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #flash_btn:pressed { background-color: #4a4a4a; }
  )");
  addItem(flash_btn);
}

ToyotaTogglesPanel::ToyotaTogglesPanel(SettingsWindow *parent) : ListWidget(parent) {

  std::vector<std::tuple<QString, QString, QString, QString>> tinkla_toggles{

    {"ToyotaUseAEBgateway",
    "Use AEB Gateway for brake control",
    "Only use this option if you have an AEB Gateway installed on your Toyota. Requires reboot.",
    "../assets/offroad/icon_settings.png",
    },
  };
  Params params;
  for (auto &[param, title, desc, icon] : tinkla_toggles) {
    auto toggle = new TinklaParamControl(param, title, desc, icon, this);
    bool locked = params.getBool((param + "Lock").toStdString());
    toggle->setEnabled(!locked);
    if (!locked) {
      connect(uiState(), &UIState::offroadTransition, toggle, &ParamControl::setEnabled);
    }
    addItem(toggle);
  };
}
