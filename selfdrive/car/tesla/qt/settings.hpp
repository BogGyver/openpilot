TinklaTogglesPanel::TinklaTogglesPanel(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *tinkla_toggles_list = new QVBoxLayout();

  QList<TinklaParamControl*> tinkla_toggles;

  
  tinkla_toggles.append(new TinklaParamControl("TinklaHso",
                                  "Enable HSO",
                                  "Enables Human Steering Override (HSO) module without disengaging OpenPilot.",
                                  "../assets/offroad/icon_warning.png",
                                  this));
  tinkla_toggles.append(new TinklaParamControl("TinklaHao",
                                  "Enable HAO",
                                  "Enables Human Steering Override (HSO) module without disengaging OpenPilot.",
                                  "../assets/offroad/icon_warning.png",
                                  this));
  tinkla_toggles.append(new TinklaParamControl("TinklaAlc",
                                  "Enable ALC",
                                  "Enables automatic lane change with just the tap of the turn signal stalk.  Your attention is required at all times to use this feature.",
                                  "../assets/offroad/icon_warning.png",
                                  this));
    tinkla_toggles.append(new TinklaParamControl("TinklaTurnScreenOff",
                                  "Turn screen off while engaged",
                                  "Keeps device screen off even when engaged. It wakes the screen any time a message is shown.",
                                  "../assets/offroad/icon_icon_settings.png",
                                  this));
  tinkla_toggles.append(new TinklaParamControl("TinklaEnablePedal",
                                  "preAP: Enable pedal interceptor",
                                  "Enables the use of the Pedal Interceptor to control the speed of your pre-AutoPilot Tesla. Requires Pedal Interceptor hardware connected to CAN2. Requires reboot.",
                                  "../assets/offroad/icon_speed_limit.png",
                                  this));
  tinkla_toggles.append(new TinklaParamControl("TinklaUseFollowACC",
                                  "preAP: Use Follow mode ACC",
                                  "Enables the use of the Follow mode ACC instead of the OP longitudinal control. Works with both CC and Pedal.",
                                  "../assets/offroad/icon_speed_limit.png",
                                  this));
  tinkla_toggles.append(new TinklaParamControl("TinklaAutoResumeACC",
                                  "preAP: AutoResume ACC",
                                  "Enables the use of the AutoResume mode ACC instead full disengagement. Works with both CC.",
                                  "../assets/offroad/icon_speed_limit.png",
                                  this));
  tinkla_toggles.append(new TinklaParamControl("TinklaUseTeslaRadar",
                                  "preAP: Use Tesla Radar",
                                  "Enables the use of the Tesla Radar for pre-AutoPilot Tesla Model S. Requires Tesla Bosch radar hardware conencted to CAN1. Requires reboot.",
                                  "../assets/offroad/icon_settings.png",
                                  this));
  tinkla_toggles.append(new TinklaParamControl("TinklaUseAPillarHarness",
                                  "preAP: Use Tesla A-pillar harness",
                                  "Enables the use of the Tesla A-pillar harness for pre-AutoPilot Tesla Model S. Requires reboot.",
                                  "../assets/offroad/icon_settings.png",
                                  this));
  for(TinklaParamControl *toggle : tinkla_toggles){
    if(tinkla_toggles_list->count() != 0){
      tinkla_toggles_list->addWidget(horizontal_line());
    }
    tinkla_toggles_list->addWidget(toggle);
  }

  setLayout(tinkla_toggles_list);
}