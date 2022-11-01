#include <string>
#include <iostream>
#include <sstream>
#include <cassert>

#include "tsettings.hpp"



#include "tcontrols.hpp"

#include "num_param.h"
#include "str_param.h"

#define TINKLA_TOGGLE 1
#define TINKLA_FLOAT 2
#define TINKLA_STRING 3

TinklaTogglesPanel::TinklaTogglesPanel(SettingsWindow *parent) : ListWidget(parent) {

  std::vector<std::tuple<QString, QString, QString, QString, QString, QString, QString, float,float,float,float,int>> tinkla_toggles{
     // param, title, desc, icon
    {"TinklaHso",
      "Enable HSO",
      "Enables Human Steering Override (HSO) module without disengaging OpenPilot.",
      "../assets/offroad/icon_warning.png",
      "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
      },
    {"TinklaHsoNumbPeriod",
      "HSO numb period",
      "The time, in seconds, to delay the reengagement of LKAS after HSO has been engaged by user by taking control of steering.",
      "../assets/offroad/icon_settings.png",
      "HSO numb period:",
      "Enter time in seconds.",
      "s",
      1.5,0.5,3.0,0.5,TINKLA_FLOAT
    },
    {"TinklaHao",
      "Enable HAO",
      "Enables Human Accelerator Override (HAO) module without disengaging OpenPilot.",
      "../assets/offroad/icon_warning.png",
      "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
      },
    {"TinklaAlc",
      "Enable ALC",
      "Enables automatic lane change with just the tap of the turn signal stalk.  Your attention is required at all times to use this feature.",
      "../assets/offroad/icon_warning.png",
      "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
      },

    {"TinklaAlcDelay",
      "ALC delay",
      "The time, in seconds, that ALC will wait and keep the turn signal on and check blind spot monitoring (when available) before automatically starting the lange change.",
      "../assets/offroad/icon_settings.png",
      "ALC delay:",
      "Enter time in seconds.",
      "s",
      2.0,1.0,3.0,0.5,TINKLA_FLOAT
    },
    {"TinklaEnableOPLong",
      "Enable OP Long Control",
      "AP1/AP2: Enables OP Long Control and disables the AP ACC. Requires 2 pandas for MS AP2/MX AP1",
      "../assets/offroad/icon_warning.png",
      "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
      },
    {"TinklaTurnScreenOff",
      "Turn screen off while engaged",
      "Keeps device screen off even when engaged. It wakes the screen any time a message is shown.",
      "../assets/offroad/icon_settings.png",
      "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
      },
    {"TinklaDebugMode",
      "Show Debug View",
      "Shows the image captured by the road camera, including detected path and lanes.",
      "../assets/offroad/icon_settings.png",
      "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
      },
    {"TinklaHideGps",
      "Hide GPS Warnings",
      "Hides the GPS warning when user doesn't care about them.",
      "../assets/offroad/icon_settings.png",
      "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
      },
    {"TinklaMapsOnLeft",
      "Maps on left side",
      "Shows the maps on the left side of the screen.",
      "../assets/offroad/icon_settings.png",
      "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
      },
    {"TinklaFlipScreen",
      "Flip screen (for RHD)",
      "Flips the screen for people driving on the wrong side of the road. Changes to C2 hardware are needed. C2 only.",
      "../assets/offroad/icon_settings.png",
      "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
      },
    {"TinklaShutdownAfter",
      "Shutdown after # of hours",
      "Shutdown device after number of hours when car is off",
      "../assets/offroad/icon_settings.png",
      "# hours:",
      "Enter # of hours to shutdown device after:",
      "",
      3.0,1.0,720.0,1.0,TINKLA_FLOAT
    },
    {"TinklaHandsOnLevel",
      "Hands on level",
      "Level at which to detect hands on wheel. Higher number means more force needed.",
      "../assets/offroad/icon_settings.png",
      "HandsOn Level:",
      "1-Light 2-Medium 3-HARD:",
      "",
      1.0,1.0,3.0,1.0,TINKLA_FLOAT
    },
    {"TinklaDevUnit",
      "Tinkla Development Unit",
      "For use by developers only.",
      "../assets/offroad/icon_settings.png",
      "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
      },
  };
  Params params;
  for (auto &[param, title, desc, icon, edit_title,edit_desc, edit_uom, val_default,val_min,val_max,val_step, field_type] : tinkla_toggles) {
    if (field_type == TINKLA_TOGGLE) {
      auto toggle = new TinklaParamControl(param, title, desc, icon, this);
      bool locked = params.getBool((param + "Lock").toStdString());
      toggle->setEnabled(!locked);
      if (!locked) {
        connect(uiState(), &UIState::offroadTransition, toggle, &ParamControl::setEnabled);
      }
      addItem(toggle);
    }
    if (field_type == TINKLA_FLOAT) {
      addItem(new NumParamControl(title, desc, edit_title,edit_desc, edit_uom, param,val_default,val_min,val_max,val_step, icon));
    }
    if (field_type == TINKLA_STRING) {
      addItem(new StrParamControl(title, desc, edit_title,edit_desc, param, QString::fromStdString(""), icon));
    }
  };
  QPushButton *flash_btn = new QPushButton("Flash Panda");
  flash_btn->setObjectName("flash_btn");

  QObject::connect(flash_btn, &QPushButton::clicked, [=](){
    QProcess::startDetached("/data/openpilot/panda/board/flashPanda");
  });

  setStyleSheet(R"(
    #flash_btn { height: 120px; border-radius: 15px; background-color: #393939; }
    #flash_btn:pressed { background-color: #4a4a4a; }
  )");
  addItem(flash_btn);

}

TeslaPreApTogglesPanel::TeslaPreApTogglesPanel(SettingsWindow *parent) : ListWidget(parent) {

  std::vector<std::tuple<QString, QString, QString, QString, QString, QString, QString, float,float,float,float,int>> tinkla_toggles{
  // param, title, desc, icon
    {"TinklaPost1916Fix",
    "Tesla software post 2019.16",
    "Use the DBC for Tesla software after 2019.16 when some of the messages changed.",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaEnablePedal",
    "Use pedal",
    "Enables the use of the Pedal Interceptor to control the speed of your pre-AutoPilot Tesla. Requires Pedal Interceptor hardware connected to CAN2. Requires reboot.",
    "../assets/offroad/icon_speed_limit.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaEnablePedalOverCC",
    "Use pedal over CC",
    "Enables the use of the Pedal Interceptor to control the speed of your pre-AutoPilot Tesla even over CC. Requires Pedal Interceptor hardware connected to CAN2. Requires reboot.",
    "../assets/offroad/icon_speed_limit.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaPedalCanZero",
    "Pedal on CAN0",
    "Uses CAN0 for pedal interceptor. Default (and safest option) is CAN2. Only enable if you know what you're doing.",
    "../assets/offroad/icon_speed_limit.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaPedalProfile",
      "Pedal Profile",
      "The profile to be used with the Tinkla Pedal Interceptor. 1 (fast accel) to 4 (slow accel)",
      "../assets/offroad/icon_speed_limit.png",
      "Pedal Interceptor Profile:",
      "Enter profile #.",
      "",
      2.0,1.0,4.0,1.0,TINKLA_FLOAT
    },
    {"TinklaAutoResumeACC",
    "AutoResume ACC",
    "Enables the use of the AutoResume mode ACC instead full disengagement. Works with both CC.",
    "../assets/offroad/icon_speed_limit.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaFollowDistance",
      "Follow Distance",
      "The number of seconds based on current speed between you and the lead vehicle.",
      "../assets/offroad/icon_speed_limit.png",
      "Follow Distance:",
      "Enter time in seconds.",
      "s",
      1.45,0.6,3.0,0.05,TINKLA_FLOAT
    },
    {"TinklaHasIcIntegration",
    "Use Tinkla Buddy",
    "Enables IC integration via Tinkla Buddy. Only enable if you have a Tinkla Buddy licensed and installed.",
    "../assets/offroad/icon_speed_limit.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaUseTeslaRadar",
    "Use Tesla Radar",
    "Enables the use of the Tesla Radar for pre-AutoPilot Tesla Model S. Requires Tesla Bosch radar hardware conencted to CAN1. Requires reboot.",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaTeslaRadarBehindNosecone",
    "Radar behind nosecone",
    "Enables the use of the Tesla Radar behind the nosecone for pre-AutoPilot Tesla Model S. Requires Tesla Bosch radar hardware conencted to CAN1. Requires reboot.",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaRadarOffset",
      "Radar offset",
      "The distance, in meters from center of car, the radar is offset.",
      "../assets/offroad/icon_settings.png",
      "Radar offset:",
      "Enter distance in meters. Positive towards left.",
      "m",
      0.0,-1.0,1.0,0.01,TINKLA_FLOAT
    },
    {"TinklaUseTeslaRadarUpsideDown",
    "Use Radar Upside Down",
    "Allows one to install the Tesla Radar upside down.",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaHasIBooster",
    "Car has iBooster",
    "Set to true if you retrofitted Tesla Model S iBooster on pre-AutoPilot cars. Requires reboot.",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaForceTeslaPreAP",
    "Force PreAP Tesla Model S ",
    "Forces the fingerprint to match a PreAP Tesla Model S. Requires reboot.",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
  };
  Params params;
  for (auto &[param, title, desc, icon, edit_title,edit_desc, edit_uom, val_default,val_min,val_max,val_step, field_type] : tinkla_toggles) {
    if (field_type == TINKLA_TOGGLE) {
      auto toggle = new TinklaParamControl(param, title, desc, icon, this);
      bool locked = params.getBool((param + "Lock").toStdString());
      toggle->setEnabled(!locked);
      if (!locked) {
        connect(uiState(), &UIState::offroadTransition, toggle, &ParamControl::setEnabled);
      }
      addItem(toggle);
    }
    if (field_type == TINKLA_FLOAT) {
      addItem(new NumParamControl(title, desc, edit_title,edit_desc, edit_uom, param,val_default,val_min,val_max,val_step, icon));
    }
    if (field_type == TINKLA_STRING) {
      addItem(new StrParamControl(title, desc, edit_title,edit_desc, param, QString::fromStdString(""), icon));
    }
  };

  QPushButton *flash_btn = new QPushButton("Flash EPAS");
  flash_btn->setObjectName("flash_btn");

  QObject::connect(flash_btn, &QPushButton::clicked, [=](){
    QProcess::startDetached("/data/openpilot/selfdrive/car/modules/teslaEpasFlasher/flashTeslaEPAS");
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

  std::vector<std::tuple<QString, QString, QString, QString, QString, QString, QString, float,float,float,float,int>> tinkla_toggles{

    {"TinklaAdjustAccWithSpeedLimit",
    "Adjust ACC max with speed limit",
    "Adjust cruise control speed limit when legal speed limit for the road changes.",
    "../assets/offroad/icon_speed_limit.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaSpeedLimitUseRelative",
    "Use relative offset",
    "Use a relative offset (percentage of speed limit).",
    "../assets/offroad/icon_speed_limit.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaSpeedLimitOffset",
      "Speed Limit Offset",
      "The speed offset vs. the legal speed limit you want ACC to apply when automatically changing with speed limit (in your car's UOM or percentage if using relative offset).",
      "../assets/offroad/icon_speed_limit.png",
      "Speed Limit Offset:",
      "Enter offset in your car's UOM",
      "",
      0.0,-5.0,20.0,1.0,TINKLA_FLOAT
    },
    {"TinklaTurnSlowdown",
    "Slowdown in Turns",
    "Reduces the car' speed in turns based on road curvature.",
    "../assets/offroad/icon_speed_limit.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaTurnSlowdownFactor",
      "Slowdown Factor in Turns",
      "The multiplier used to compute the safe speed to take a turn. 0.8 is slower, 1.2 is faster.",
      "../assets/offroad/icon_speed_limit.png",
      "Slowdown Factor in Turns:",
      "Enter the slowdown multiplier:",
      "",
      0.95,0.8,1.2,0.01,TINKLA_FLOAT
    },
    {"TinklaBrakeFactor",
      "Braking Factor",
      "The multiplier used to compute the Tesla braking power. 0.8 is slower, 1.2 is faster.",
      "../assets/offroad/icon_speed_limit.png",
      "Braking Factor:",
      "Enter the braking multiplier:",
      "",
      1.0,0.8,1.2,0.01,TINKLA_FLOAT
    },
    {"TinklaSpeedMadMax",
    "Use MadMax Mode",
    "Use faster accel limits.",
    "../assets/offroad/icon_speed_limit.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaUseLongControlData",
    "Use LongControl data",
    "Uses data from Tesla fleet to control speed based on maps. Requires AP.",
    "../assets/offroad/icon_speed_limit.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaTeslaRadarIgnoreSGUError",
    "Ignore Radar Errors",
    "Ignore Tesla Radar errors about calibration. ",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaAPForceFingerprint",
      "Force Tesla AP Fingerprint",
      "Forces fingerprint for a specific model of Tesla with AutoPilot. This should not be used for preAP Tesla Model S.",
      "../assets/offroad/icon_settings.png",
      "Fingerprint:",
      "Enter a valid fingerprint name. Check WiKi for values.",
      "",
      0.0,0.0,0.0,0.0,TINKLA_STRING
    },
    {"TinklaDisableStartStopSounds",
    "Disable Main Sounds",
    "Disables the device from playing the Engagement and Disengagement sounds. To be used when the car will generate these sounds by itself. Prompt and Warning sounds will still be played.",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
    {"TinklaDisablePromptSounds",
    "Disable Prompt Sounds",
    "Disables the device from playing the Prompt sounds. To be used when the car will generate these sounds by itself.  Engagement/Disengagement and Warning sounds will still be played.",
    "../assets/offroad/icon_settings.png",
    "","","",0.0,0.0,0.0,0.0, TINKLA_TOGGLE
    },
  };
  Params params;
  for (auto &[param, title, desc, icon, edit_title,edit_desc, edit_uom, val_default,val_min,val_max,val_step, field_type] : tinkla_toggles) {
    if (field_type == TINKLA_TOGGLE) {
      auto toggle = new TinklaParamControl(param, title, desc, icon, this);
      bool locked = params.getBool((param + "Lock").toStdString());
      toggle->setEnabled(!locked);
      if (!locked) {
        connect(uiState(), &UIState::offroadTransition, toggle, &ParamControl::setEnabled);
      }
      addItem(toggle);
    }
    if (field_type == TINKLA_FLOAT) {
      addItem(new NumParamControl(title, desc, edit_title,edit_desc, edit_uom, param,val_default,val_min,val_max,val_step, icon));
    }
    if (field_type == TINKLA_STRING) {
      addItem(new StrParamControl(title, desc, edit_title,edit_desc, param, QString::fromStdString(""), icon));
    }
  };
}
