#include <string>
#include <iostream>
#include <sstream>
#include <cassert>

#include "tsettings.hpp"



#include "tcontrols.hpp"


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

TeslaTogglesPanel::TeslaTogglesPanel(SettingsWindow *parent) : ListWidget(parent) {
  
  std::vector<std::tuple<QString, QString, QString, QString>> tinkla_toggles{
  // param, title, desc, icon
    {"TinklaPost1916Fix",
    "Tesla software 2019.16 or newer",
    "Use the DBC for Tesla software after 2019.16 when some of the messages changed.",
    "../assets/offroad/icon_settings.png",
    },
    {"TinklaAllowHumanLong",
    "preAP: Allow OP to just control LKAS",
    "Allows the user to perform longitudinal control via accel/brake without disengaging OP. Requires reboot.",
    "../assets/offroad/icon_speed_limit.png",
    },
    {"TinklaEnablePedal",
    "preAP: Enable pedal interceptor",
    "Enables the use of the Pedal Interceptor to control the speed of your pre-AutoPilot Tesla. Requires Pedal Interceptor hardware connected to CAN2. Requires reboot.",
    "../assets/offroad/icon_speed_limit.png",
    },
    {"TinklaUseFollowACC",
    "preAP: Use Follow mode ACC",
    "Enables the use of the Follow mode ACC instead of the OP longitudinal control. Works with both CC and Pedal.",
    "../assets/offroad/icon_speed_limit.png",
    },
    {"TinklaAutoResumeACC",
    "preAP: AutoResume ACC",
    "Enables the use of the AutoResume mode ACC instead full disengagement. Works with both CC.",
    "../assets/offroad/icon_speed_limit.png",
    },
    {"TinklaUseTeslaRadar",
    "preAP: Use Tesla Radar",
    "Enables the use of the Tesla Radar for pre-AutoPilot Tesla Model S. Requires Tesla Bosch radar hardware conencted to CAN1. Requires reboot.",
    "../assets/offroad/icon_settings.png",
    },
    {"TinklaHasIBooster",
    "preAP: Car has iBooster retrofitted",
    "Set to true if you retrofitted Tesla Model S iBooster on pre-AutoPilot cars. Requires reboot.",
    "../assets/offroad/icon_settings.png",
    },
    {"TinklaForceTeslaPreAP",
    "preAP: Force Fingerprint PreAP Tesla Model S ",
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
