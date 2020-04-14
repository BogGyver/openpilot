#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/resource.h>

#include <czmq.h>

#include "common/util.h"
#include "common/timing.h"
#include "common/swaglog.h"
#include "common/touch.h"
#include "common/visionimg.h"
#include "common/params.h"
#include "ui.hpp"
#include "sound.hpp"

#if !defined(QCOM) && !defined(QCOM2)
#include <X11/Xlib.h>
int force_rez = 0;
int force_rez_w = 0;
int force_rez_h = 0;
#endif

#include "bbui.h"

static int last_brightness = -1;
static void set_brightness(UIState *s, int brightness) {
  if (last_brightness != brightness && (s->awake || brightness == 0)) {
    FILE *f = fopen("/sys/class/leds/lcd-backlight/brightness", "wb");
    if (f != NULL) {
      fprintf(f, "%d", brightness);
      fclose(f);
      last_brightness = brightness;
    }
  }
}

static void set_awake(UIState *s, bool awake) {
#ifdef QCOM
  if (awake) {
    // 30 second timeout at 30 fps
    if (((s->b.tri_state_switch == 3) || (s->b.keepEonOff)) && !s->b.recording) {
      s->awake_timeout = 3*30;
    } else {
      s->awake_timeout = 30*30;
    }
  }
  if (s->awake != awake) {
    s->awake = awake;

    // TODO: replace command_awake and command_sleep with direct calls to android
    if (awake) {
      LOGW("awake normal");
      system("service call window 18 i32 1");  // enable event processing
      framebuffer_set_power(s->fb, HWC_POWER_MODE_NORMAL);
    } else {
      LOGW("awake off");
      set_brightness(s, 0);
      system("service call window 18 i32 0");  // disable event processing
      framebuffer_set_power(s->fb, HWC_POWER_MODE_OFF);
    }
  }
#else
  // computer UI doesn't sleep
  s->awake = true;
#endif
}

#include "dashcam.h"
#include "tbui.h"

static void navigate_to_settings(UIState *s) {
#ifdef QCOM
  system("am broadcast -a 'ai.comma.plus.SidebarSettingsTouchUpInside'");
#else
  // computer UI doesn't have offroad settings
#endif
}

static void navigate_to_home(UIState *s) {
#ifdef QCOM
  system("am broadcast -a 'ai.comma.plus.HomeButtonTouchUpInside'");
#else
  // computer UI doesn't have offroad home
#endif
}

static void handle_sidebar_touch(UIState *s, int touch_x, int touch_y) {
  if (!s->scene.uilayout_sidebarcollapsed && touch_x <= sbr_w) {
    if (touch_x >= settings_btn_x && touch_x < (settings_btn_x + settings_btn_w)
      && touch_y >= settings_btn_y && touch_y < (settings_btn_y + settings_btn_h)) {
      navigate_to_settings(s);
    }
    if (touch_x >= home_btn_x && touch_x < (home_btn_x + home_btn_w)
      && touch_y >= home_btn_y && touch_y < (home_btn_y + home_btn_h)) {
      navigate_to_home(s);
      if (s->vision_connected) {
        s->scene.uilayout_sidebarcollapsed = true;
      }
    }
  }
}

static void handle_vision_touch(UIState *s, int touch_x, int touch_y) {
  if (s->vision_connected && (touch_x >= s->scene.ui_viz_rx - bdr_s)
    && (s->active_app != cereal_UiLayoutState_App_settings)) {
    s->scene.uilayout_sidebarcollapsed = !s->scene.uilayout_sidebarcollapsed;
  }
}

volatile sig_atomic_t do_exit = 0;
static void set_do_exit(int sig) {
  do_exit = 1;
}

static void read_param_bool(bool* param, const char* param_name) {
  char *s;
  const int result = read_db_value(NULL, param_name, &s, NULL);
  if (result == 0) {
    *param = s[0] == '1';
    free(s);
  }
}

static void read_param_float(float* param, const char* param_name) {
  char *s;
  const int result = read_db_value(NULL, param_name, &s, NULL);
  if (result == 0) {
    *param = strtod(s, NULL);
    free(s);
  }
}

static void read_param_bool_timeout(bool* param, const char* param_name, int* timeout) {
  if (*timeout > 0){
    (*timeout)--;
  } else {
    read_param_bool(param, param_name);
    *timeout = 2 * UI_FREQ; // 0.5Hz
  }
}

static void read_param_float_timeout(float* param, const char* param_name, int* timeout) {
  if (*timeout > 0){
    (*timeout)--;
  } else {
    read_param_float(param, param_name);
    *timeout = 2 * UI_FREQ; // 0.5Hz
  }
}

static void ui_init(UIState *s) {
  memset(s, 0, sizeof(UIState));

  pthread_mutex_init(&s->lock, NULL);
  pthread_cond_init(&s->bg_cond, NULL);

  s->ctx = Context::create();
  s->model_sock = SubSocket::create(s->ctx, "model");
  s->controlsstate_sock = SubSocket::create(s->ctx, "controlsState");
  s->uilayout_sock = SubSocket::create(s->ctx, "uiLayoutState");
  s->livecalibration_sock = SubSocket::create(s->ctx, "liveCalibration");
  s->radarstate_sock = SubSocket::create(s->ctx, "radarState");
  s->thermal_sock = SubSocket::create(s->ctx, "thermal");
  s->health_sock = SubSocket::create(s->ctx, "health");
  s->ubloxgnss_sock = SubSocket::create(s->ctx, "ubloxGnss");

  assert(s->model_sock != NULL);
  assert(s->controlsstate_sock != NULL);
  assert(s->uilayout_sock != NULL);
  assert(s->livecalibration_sock != NULL);
  assert(s->radarstate_sock != NULL);
  assert(s->thermal_sock != NULL);
  assert(s->health_sock != NULL);
  assert(s->ubloxgnss_sock != NULL);

  s->poller = Poller::create({
                              s->model_sock,
                              s->controlsstate_sock,
                              s->uilayout_sock,
                              s->livecalibration_sock,
                              s->radarstate_sock,
                              s->thermal_sock,
                              s->health_sock,
                              s->ubloxgnss_sock
                             });

#ifdef SHOW_SPEEDLIMIT
  s->map_data_sock = SubSocket::create(s->ctx, "liveMapData");
  assert(s->map_data_sock != NULL);
  s->poller->registerSocket(s->map_data_sock);
#endif

  s->ipc_fd = -1;

  // init display
#if defined(QCOM) || defined(QCOM2)
  s->fb = framebuffer_init("ui", 0x00010000, true, &s->fb_w, &s->fb_h);
#else
  s->fb = framebuffer_init_linux("ui", 0x00010000, true, &s->fb_w, &s->fb_h,bb_mouse_event_handler);
#endif 
  assert(s->fb);

  set_awake(s, true);

  s->model_changed = false;
  s->livempc_or_radarstate_changed = false;
#if defined(QCOM) || defined(QCOM2)
  s->b.scr_w = vwp_w;
  s->b.scr_h = vwp_h;
  s->b.scr_scale_x = 1.0f;
  s->b.scr_scale_y = 1.0f;
  s->b.scr_device_factor = 1.0f;
  s->b.scr_scissor_offset = 0.0f;
#else
  //scale
  if (force_rez == 1) {
    s->b.scr_w = force_rez_w;
    s->b.scr_h = force_rez_h;
    printf("Forcing rezolution to %d x %d \n",force_rez_w,force_rez_h);

  } else {
    s->b.scr_display = XOpenDisplay(NULL);
    Screen*  xorg_s = DefaultScreenOfDisplay(s->b.scr_display);
    s->b.scr_w = xorg_s->width;
    s->b.scr_h = xorg_s->height;
  }
  s->b.scr_scale_x = (float)(s->b.scr_w) / (float)(vwp_w);
  s->b.scr_scale_y = (float)(s->b.scr_h) / (float)(vwp_h);
  s->b.scr_device_factor =  (float)(1920) / (float)(s->b.scr_w);
  s->b.scr_scissor_offset = (float)(s->b.scr_w * 3) / 4.0f - (float)(s->b.scr_h);
  mouse_ui_state = s;
#endif
  ui_nvg_init(s);
}

static void ui_init_vision(UIState *s, const VisionStreamBufs back_bufs,
                           int num_back_fds, const int *back_fds,
                           const VisionStreamBufs front_bufs, int num_front_fds,
                           const int *front_fds) {
  const VisionUIInfo ui_info = back_bufs.buf_info.ui_info;

  assert(num_back_fds == UI_BUF_COUNT);
  assert(num_front_fds == UI_BUF_COUNT);

  vipc_bufs_load(s->bufs, &back_bufs, num_back_fds, back_fds);
  vipc_bufs_load(s->front_bufs, &front_bufs, num_front_fds, front_fds);

  s->cur_vision_idx = -1;
  s->cur_vision_front_idx = -1;

  s->scene = (UIScene){
      .frontview = getenv("FRONTVIEW") != NULL,
      .fullview = getenv("FULLVIEW") != NULL,
      .transformed_width = ui_info.transformed_width,
      .transformed_height = ui_info.transformed_height,
      .front_box_x = ui_info.front_box_x,
      .front_box_y = ui_info.front_box_y,
      .front_box_width = ui_info.front_box_width,
      .front_box_height = ui_info.front_box_height,
      .world_objects_visible = false,  // Invisible until we receive a calibration message.
      .gps_planner_active = false,
  };

  s->rgb_width = back_bufs.width;
  s->rgb_height = back_bufs.height;
  s->rgb_stride = back_bufs.stride;
  s->rgb_buf_len = back_bufs.buf_len;

  s->rgb_front_width = front_bufs.width;
  s->rgb_front_height = front_bufs.height;
  s->rgb_front_stride = front_bufs.stride;
  s->rgb_front_buf_len = front_bufs.buf_len;

  s->rgb_transform = (mat4){{
    2.0f/s->rgb_width, 0.0f, 0.0f, -1.0f,
    0.0f, 2.0f/s->rgb_height, 0.0f, -1.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f,
  }};

  read_param_float(&s->speed_lim_off, "SpeedLimitOffset");
  read_param_bool(&s->is_metric, "IsMetric");
  read_param_bool(&s->longitudinal_control, "LongitudinalControl");
  read_param_bool(&s->limit_set_speed, "LimitSetSpeed");

  // Set offsets so params don't get read at the same time
  s->longitudinal_control_timeout = UI_FREQ / 3;
  s->is_metric_timeout = UI_FREQ / 2;
  s->limit_set_speed_timeout = UI_FREQ;
}

static PathData read_path(cereal_ModelData_PathData_ptr pathp) {
  PathData ret = {0};

  struct cereal_ModelData_PathData pathd;
  cereal_read_ModelData_PathData(&pathd, pathp);

  ret.prob = pathd.prob;
  ret.std = pathd.std;

  capn_list32 polyp = pathd.poly;
  capn_resolve(&polyp.p);
  for (int i = 0; i < POLYFIT_DEGREE; i++) {
    ret.poly[i] = capn_to_f32(capn_get32(polyp, i));
  }

  // Compute points locations
  for (int i = 0; i < MODEL_PATH_DISTANCE; i++) {
    ret.points[i] = ret.poly[0] * (i*i*i) + ret.poly[1] * (i*i)+ ret.poly[2] * i + ret.poly[3];
  }

  return ret;
}

static ModelData read_model(cereal_ModelData_ptr modelp) {
  struct cereal_ModelData modeld;
  cereal_read_ModelData(&modeld, modelp);

  ModelData d = {0};

  d.path = read_path(modeld.path);
  d.left_lane = read_path(modeld.leftLane);
  d.right_lane = read_path(modeld.rightLane);

  struct cereal_ModelData_LeadData leadd;
  cereal_read_ModelData_LeadData(&leadd, modeld.lead);
  d.lead = (LeadData){
      .dist = leadd.dist, .prob = leadd.prob, .std = leadd.std,
  };

  return d;
}

static void update_status(UIState *s, int status) {
  //BB Variable for the old status
  int old_status = s->status;
  if (s->status != status) {
    s->status = status;
    // wake up bg thread to change
    pthread_cond_signal(&s->bg_cond);
  }
}


void handle_message(UIState *s, Message * msg) {
  struct capn ctx;
  capn_init_mem(&ctx, (uint8_t*)msg->getData(), msg->getSize(), 0);

  cereal_Event_ptr eventp;
  eventp.p = capn_getp(capn_root(&ctx), 0, 1);
  struct cereal_Event eventd;
  cereal_read_Event(&eventd, eventp);

  int bts = bb_get_button_status(s,(char *)"sound");
  if (eventd.which == cereal_Event_controlsState) {
    struct cereal_ControlsState datad;
    cereal_read_ControlsState(&datad, eventd.controlsState);

    s->controls_timeout = 1 * UI_FREQ;
    s->controls_seen = true;

    if (datad.vCruise != s->scene.v_cruise) {
      s->scene.v_cruise_update_ts = eventd.logMonoTime;
    }
    s->scene.v_cruise = datad.vCruise;
    s->scene.v_ego = datad.vEgo;
    s->scene.curvature = datad.curvature;
    //BB get angles
    s->b.angleSteers = datad.angleSteers;
    s->b.angleSteersDes = datad.angleSteersDes;
    //BB END
    s->scene.engaged = datad.enabled;
    s->scene.engageable = datad.engageable;
    s->scene.gps_planner_active = datad.gpsPlannerActive;
    s->scene.monitoring_active = datad.driverMonitoringOn;

    s->scene.frontview = datad.rearViewCam;

    s->scene.decel_for_model = datad.decelForModel;

    if (datad.alertSound != cereal_CarControl_HUDControl_AudibleAlert_none && datad.alertSound != s->alert_sound) {
      if (s->alert_sound != cereal_CarControl_HUDControl_AudibleAlert_none) {
        stop_alert_sound(s->alert_sound);
      }
      if (bts !=0) {
        play_alert_sound(datad.alertSound);
      }

      s->alert_sound = datad.alertSound;
      snprintf(s->alert_type, sizeof(s->alert_type), "%s", datad.alertType.str);
    } else if ((!datad.alertSound || datad.alertSound == cereal_CarControl_HUDControl_AudibleAlert_none)
                  && s->alert_sound != cereal_CarControl_HUDControl_AudibleAlert_none) {
      stop_alert_sound(s->alert_sound);
      s->alert_type[0] = '\0';
      s->alert_sound = cereal_CarControl_HUDControl_AudibleAlert_none;
    }

    if (datad.alertText1.str) {
      snprintf(s->scene.alert_text1, sizeof(s->scene.alert_text1), "%s", datad.alertText1.str);
    } else {
      s->scene.alert_text1[0] = '\0';
    }
    if (datad.alertText2.str) {
      snprintf(s->scene.alert_text2, sizeof(s->scene.alert_text2), "%s", datad.alertText2.str);
    } else {
      s->scene.alert_text2[0] = '\0';
    }
    s->scene.awareness_status = datad.awarenessStatus;

    s->scene.alert_ts = eventd.logMonoTime;

    s->scene.alert_size = datad.alertSize;
    if (datad.alertSize == cereal_ControlsState_AlertSize_none) {
      s->alert_size = ALERTSIZE_NONE;
    } else if (datad.alertSize == cereal_ControlsState_AlertSize_small) {
      s->alert_size = ALERTSIZE_SMALL;
    } else if (datad.alertSize == cereal_ControlsState_AlertSize_mid) {
      s->alert_size = ALERTSIZE_MID;
    } else if (datad.alertSize == cereal_ControlsState_AlertSize_full) {
      s->alert_size = ALERTSIZE_FULL;
    }

    if (s->status != STATUS_STOPPED) {
      if (datad.alertStatus == cereal_ControlsState_AlertStatus_userPrompt) {
        update_status(s, STATUS_WARNING);
      } else if (datad.alertStatus == cereal_ControlsState_AlertStatus_critical) {
        update_status(s, STATUS_ALERT);
      } else if (datad.enabled) {
        update_status(s, STATUS_ENGAGED);
      } else {
        update_status(s, STATUS_DISENGAGED);
      }
    }

    s->scene.alert_blinkingrate = datad.alertBlinkingRate;
    if (datad.alertBlinkingRate > 0.) {
      if (s->alert_blinked) {
        if (s->alert_blinking_alpha > 0.0 && s->alert_blinking_alpha < 1.0) {
          s->alert_blinking_alpha += (0.05*datad.alertBlinkingRate);
        } else {
          s->alert_blinked = false;
        }
      } else {
        if (s->alert_blinking_alpha > 0.25) {
          s->alert_blinking_alpha -= (0.05*datad.alertBlinkingRate);
        } else {
          s->alert_blinking_alpha += 0.25;
          s->alert_blinked = true;
        }
      }
    }
  } else if (eventd.which == cereal_Event_radarState) {
    struct cereal_RadarState datad;
    cereal_read_RadarState(&datad, eventd.radarState);
    struct cereal_RadarState_LeadData leaddatad;
    cereal_read_RadarState_LeadData(&leaddatad, datad.leadOne);
    s->scene.lead_status = leaddatad.status;
    s->scene.lead_d_rel = leaddatad.dRel;
    s->scene.lead_y_rel = leaddatad.yRel;
    s->scene.lead_v_rel = leaddatad.vRel;
    cereal_read_RadarState_LeadData(&leaddatad, datad.leadTwo);
    s->scene.lead_status2 = leaddatad.status;
    s->scene.lead_d_rel2 = leaddatad.dRel;
    s->scene.lead_y_rel2 = leaddatad.yRel;
    s->scene.lead_v_rel2 = leaddatad.vRel;
    s->livempc_or_radarstate_changed = true;
  } else if (eventd.which == cereal_Event_liveCalibration) {
    s->scene.world_objects_visible = true;
    struct cereal_LiveCalibrationData datad;
    cereal_read_LiveCalibrationData(&datad, eventd.liveCalibration);

    capn_list32 extrinsicl = datad.extrinsicMatrix;
    capn_resolve(&extrinsicl.p);  // is this a bug?
    for (int i = 0; i < 3 * 4; i++) {
      s->scene.extrinsic_matrix.v[i] =
          capn_to_f32(capn_get32(extrinsicl, i));
    }
  } else if (eventd.which == cereal_Event_model) {
    s->scene.model = read_model(eventd.model);
    s->model_changed = true;
  } else if (eventd.which == cereal_Event_liveMpc) {
    struct cereal_LiveMpcData datad;
    cereal_read_LiveMpcData(&datad, eventd.liveMpc);

    capn_list32 x_list = datad.x;
    capn_resolve(&x_list.p);

    for (int i = 0; i < 50; i++){
      s->scene.mpc_x[i] = capn_to_f32(capn_get32(x_list, i));
    }

    capn_list32 y_list = datad.y;
    capn_resolve(&y_list.p);

    for (int i = 0; i < 50; i++){
      s->scene.mpc_y[i] = capn_to_f32(capn_get32(y_list, i));
    }
    s->livempc_or_radarstate_changed = true;
  } else if (eventd.which == cereal_Event_uiLayoutState) {
    struct cereal_UiLayoutState datad;
    cereal_read_UiLayoutState(&datad, eventd.uiLayoutState);
    s->active_app = datad.activeApp;
    s->scene.uilayout_sidebarcollapsed = datad.sidebarCollapsed;
  } else if (eventd.which == cereal_Event_liveMapData) {
    struct cereal_LiveMapData datad;
    cereal_read_LiveMapData(&datad, eventd.liveMapData);
    s->scene.map_valid = datad.mapValid;
  } else if (eventd.which == cereal_Event_thermal) {
    struct cereal_ThermalData datad;
    cereal_read_ThermalData(&datad, eventd.thermal);

    s->scene.networkType = datad.networkType;
    s->scene.networkStrength = datad.networkStrength;
    s->scene.batteryPercent = datad.batteryPercent;
    snprintf(s->scene.batteryStatus, sizeof(s->scene.batteryStatus), "%s", datad.batteryStatus.str);
    s->scene.freeSpace = datad.freeSpace;
    s->scene.thermalStatus = datad.thermalStatus;
    s->scene.paTemp = datad.pa0;
    //BB CPU TEMP
    s->b.maxCpuTemp=datad.cpu0;
    if (s->b.maxCpuTemp<datad.cpu1)
    {
        s->b.maxCpuTemp=datad.cpu1;
    }
    else if (s->b.maxCpuTemp<datad.cpu2)
    {
        s->b.maxCpuTemp=datad.cpu2;
    }
    else if (s->b.maxCpuTemp<datad.cpu3)
    {
        s->b.maxCpuTemp=datad.cpu3;
    }
    s->b.maxBatTemp=datad.bat;
    s->b.freeSpace=datad.freeSpace;
    s->b.batteryPercent=datad.batteryPercent;
    s->b.chargingEnabled=!datad.chargingDisabled;
    s->b.fanSpeed=datad.fanSpeed;
    //BB END CPU TEMP
  } else if (eventd.which == cereal_Event_ubloxGnss) {
    struct cereal_UbloxGnss datad;
    cereal_read_UbloxGnss(&datad, eventd.ubloxGnss);
    if (datad.which == cereal_UbloxGnss_measurementReport) {
      struct cereal_UbloxGnss_MeasurementReport reportdatad;
      cereal_read_UbloxGnss_MeasurementReport(&reportdatad, datad.measurementReport);
      s->scene.satelliteCount = reportdatad.numMeas;
    }
  } else if (eventd.which == cereal_Event_health) {
    struct cereal_HealthData datad;
    cereal_read_HealthData(&datad, eventd.health);

    s->scene.hwType = datad.hwType;
    s->hardware_timeout = 5*30; // 5 seconds at 30 fps
  }
  capn_free(&ctx);
}

static void check_messages(UIState *s) {
  while(true) {
    auto polls = s->poller->poll(0);

    if (polls.size() == 0)
      break;

    for (auto sock : polls){
      Message * msg = sock->receive();
      if (msg == NULL) continue;

      handle_message(s, msg);

      delete msg;
    }
  }
}

static void ui_update(UIState *s) {
  int err;

  if (s->vision_connect_firstrun) {
    // cant run this in connector thread because opengl.
    // do this here for now in lieu of a run_on_main_thread event

    for (int i=0; i<UI_BUF_COUNT; i++) {
      if(s->khr[i] != NULL) {
        visionimg_destroy_gl(s->khr[i], s->priv_hnds[i]);
        glDeleteTextures(1, &s->frame_texs[i]);
      }

      VisionImg img = {
        .fd = s->bufs[i].fd,
        .format = VISIONIMG_FORMAT_RGB24,
        .width = s->rgb_width,
        .height = s->rgb_height,
        .stride = s->rgb_stride,
        .bpp = 3,
        .size = s->rgb_buf_len,
      };
      #ifndef QCOM
        s->priv_hnds[i] = s->bufs[i].addr;
      #endif
      s->frame_texs[i] = visionimg_to_gl(&img, &s->khr[i], &s->priv_hnds[i]);

      glBindTexture(GL_TEXTURE_2D, s->frame_texs[i]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

      // BGR
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, GL_GREEN);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, GL_RED);
    }

    for (int i=0; i<UI_BUF_COUNT; i++) {
      if(s->khr_front[i] != NULL) {
        visionimg_destroy_gl(s->khr_front[i], s->priv_hnds_front[i]);
        glDeleteTextures(1, &s->frame_front_texs[i]);
      }

      VisionImg img = {
        .fd = s->front_bufs[i].fd,
        .format = VISIONIMG_FORMAT_RGB24,
        .width = s->rgb_front_width,
        .height = s->rgb_front_height,
        .stride = s->rgb_front_stride,
        .bpp = 3,
        .size = s->rgb_front_buf_len,
      };
      #ifndef QCOM
        s->priv_hnds_front[i] = s->bufs[i].addr;
      #endif
      s->frame_front_texs[i] = visionimg_to_gl(&img, &s->khr_front[i], &s->priv_hnds_front[i]);

      glBindTexture(GL_TEXTURE_2D, s->frame_front_texs[i]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

      // BGR
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, GL_GREEN);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, GL_RED);
    }

    assert(glGetError() == GL_NO_ERROR);

    s->scene.uilayout_sidebarcollapsed = true;
    s->scene.ui_viz_rx = (box_x-sbr_w+bdr_s*2);
    s->scene.ui_viz_rw = (box_w+sbr_w-(bdr_s*2));
    s->scene.ui_viz_ro = 0;

    s->vision_connect_firstrun = false;

    s->alert_blinking_alpha = 1.0;
    s->alert_blinked = false;
  }

  zmq_pollitem_t polls[1] = {{0}};
  // Take an rgb image from visiond if there is one
  while(true) {
    assert(s->ipc_fd >= 0);
    polls[0].fd = s->ipc_fd;
    polls[0].events = ZMQ_POLLIN;
    #ifdef UI_60FPS
      // uses more CPU in both UI and surfaceflinger
      // 16% / 21%
      int ret = zmq_poll(polls, 1, 1);
    #else
      // 9% / 13% = a 14% savings
      int ret = zmq_poll(polls, 1, 1000);
    #endif
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;

      LOGE("poll failed (%d - %d)", ret, errno);
      close(s->ipc_fd);
      s->ipc_fd = -1;
      s->vision_connected = false;
      return;
    } else if (ret == 0) {
      break;
    }
    // vision ipc event
    VisionPacket rp;
    err = vipc_recv(s->ipc_fd, &rp);
    if (err <= 0) {
      LOGW("vision disconnected");
      close(s->ipc_fd);
      s->ipc_fd = -1;
      s->vision_connected = false;
      return;
    }
    if (rp.type == VIPC_STREAM_ACQUIRE) {
      bool front = rp.d.stream_acq.type == VISION_STREAM_RGB_FRONT;
      int idx = rp.d.stream_acq.idx;

      int release_idx;
      if (front) {
        release_idx = s->cur_vision_front_idx;
      } else {
        release_idx = s->cur_vision_idx;
      }
      if (release_idx >= 0) {
        VisionPacket rep = {
          .type = VIPC_STREAM_RELEASE,
          .d = { .stream_rel = {
            .type = rp.d.stream_acq.type,
            .idx = release_idx,
          }},
        };
        vipc_send(s->ipc_fd, &rep);
      }

      if (front) {
        assert(idx < UI_BUF_COUNT);
        s->cur_vision_front_idx = idx;
      } else {
        assert(idx < UI_BUF_COUNT);
        s->cur_vision_idx = idx;
        // printf("v %d\n", ((uint8_t*)s->bufs[idx].addr)[0]);
      }
      //BB merge: not sure where this will go for now
      //if (((awake) && (s->b.tri_state_switch != 3) && (!s->b.keepEonOff)) || (s->b.recording)){
      //  set_awake(s, true);
      //}
    } else {
      assert(false);
    }
    break;
  }
  // peek and consume all events in the zmq queue, then return.
  check_messages(s);
}

static int vision_subscribe(int fd, VisionPacket *rp, VisionStreamType type) {
  int err;
  LOGW("vision_subscribe type:%d", type);

  VisionPacket p1 = {
    .type = VIPC_STREAM_SUBSCRIBE,
    .d = { .stream_sub = { .type = type, .tbuffer = true, }, },
  };
  err = vipc_send(fd, &p1);
  if (err < 0) {
    close(fd);
    return 0;
  }

  do {
    err = vipc_recv(fd, rp);
    if (err <= 0) {
      close(fd);
      return 0;
    }

    // release what we aren't ready for yet
    if (rp->type == VIPC_STREAM_ACQUIRE) {
      VisionPacket rep = {
        .type = VIPC_STREAM_RELEASE,
        .d = { .stream_rel = {
          .type = rp->d.stream_acq.type,
          .idx = rp->d.stream_acq.idx,
        }},
      };
      vipc_send(fd, &rep);
    }
  } while (rp->type != VIPC_STREAM_BUFS || rp->d.stream_bufs.type != type);

  return 1;
}

static void* vision_connect_thread(void *args) {
  int err;
  set_thread_name("vision_connect");

  UIState *s = (UIState*)args;
  while (!do_exit) {
    usleep(100000);
    pthread_mutex_lock(&s->lock);
    bool connected = s->vision_connected;
    pthread_mutex_unlock(&s->lock);
    if (connected) continue;

    int fd = vipc_connect();
    if (fd < 0) continue;

    VisionPacket back_rp, front_rp;
    if (!vision_subscribe(fd, &back_rp, VISION_STREAM_RGB_BACK)) continue;
    if (!vision_subscribe(fd, &front_rp, VISION_STREAM_RGB_FRONT)) continue;

    pthread_mutex_lock(&s->lock);
    assert(!s->vision_connected);
    s->ipc_fd = fd;

    ui_init_vision(s,
                   back_rp.d.stream_bufs, back_rp.num_fds, back_rp.fds,
                   front_rp.d.stream_bufs, front_rp.num_fds, front_rp.fds);

    s->vision_connected = true;
    s->vision_connect_firstrun = true;

    // Drain sockets
    while (true){
      auto polls = s->poller->poll(0);
      if (polls.size() == 0)
        break;

      for (auto sock : polls){
        Message * msg = sock->receive();
        if (msg == NULL) continue;
        delete msg;
      }
    }

    pthread_mutex_unlock(&s->lock);
  }
  return NULL;
}

#ifdef QCOM

#include <cutils/properties.h>
#include <hardware/sensors.h>
#include <utils/Timers.h>

static void* light_sensor_thread(void *args) {
  int err;
  set_thread_name("light_sensor");

  UIState *s = (UIState*)args;
  s->light_sensor = 0.0;

  struct sensors_poll_device_t* device;
  struct sensors_module_t* module;

  hw_get_module(SENSORS_HARDWARE_MODULE_ID, (hw_module_t const**)&module);
  sensors_open(&module->common, &device);

  // need to do this
  struct sensor_t const* list;
  int count = module->get_sensors_list(module, &list);

  int SENSOR_LIGHT = 7;

  err = device->activate(device, SENSOR_LIGHT, 0);
  if (err != 0) goto fail;
  err = device->activate(device, SENSOR_LIGHT, 1);
  if (err != 0) goto fail;

  device->setDelay(device, SENSOR_LIGHT, ms2ns(100));

  while (!do_exit) {
    static const size_t numEvents = 1;
    sensors_event_t buffer[numEvents];

    int n = device->poll(device, buffer, numEvents);
    if (n < 0) {
      LOG_100("light_sensor_poll failed: %d", n);
    }
    if (n > 0) {
      s->light_sensor = buffer[0].light;
    }
  }

  return NULL;

fail:
  LOGE("LIGHT SENSOR IS MISSING");
  s->light_sensor = 255;
  return NULL;
}

static void* bg_thread(void* args) {
  UIState *s = (UIState*)args;
  set_thread_name("bg");
#if defined(QCOM) || defined(QCOM2)
  FramebufferState *bg_fb = framebuffer_init("bg", 0x00001000, false, NULL, NULL);
#else
  FramebufferState *bg_fb = framebuffer_init_linux("bg", 0x00001000, false, NULL, NULL, NULL);
#endif
  assert(bg_fb);

  int bg_status = -1;
  while(!do_exit) {
    pthread_mutex_lock(&s->lock);
    //BB Change of background based on our color
    int actual_status = bb_get_status(s);
    if (bg_status == actual_status) {
      // will always be signaled if it changes?
      pthread_cond_wait(&s->bg_cond, &s->lock);
    }
    bg_status = actual_status;
    //BB End of background color change
    pthread_mutex_unlock(&s->lock);

    assert(bg_status < ARRAYSIZE(bg_colors));
    const uint8_t *color = bg_colors[bg_status];

    glClearColor(color[0]/256.0, color[1]/256.0, color[2]/256.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);

    framebuffer_swap(bg_fb);
  }

  return NULL;
}

#endif

int is_leon() {
  #define MAXCHAR 1000
  FILE *fp;
  char str[MAXCHAR];
  const char* filename = "/proc/cmdline";

  fp = fopen(filename, "r");
  if (fp == NULL){
    printf("Could not open file %s",filename);
    return 0;
  }
  fgets(str, MAXCHAR, fp);
  fclose(fp);
  return strstr(str, "letv") != NULL;
}



int main(int argc, char* argv[]) {
  int err;
  setpriority(PRIO_PROCESS, 0, -14);

  zsys_handler_set(NULL);
  signal(SIGINT, (sighandler_t)set_do_exit);
#if !defined(QCOM) && !defined(QCOM2)
  if (argc == 4) {
    if (strcmp(argv[1],"-rez")==0) {
	 force_rez = 1;
	 force_rez_w = atoi(argv[2]);
	 force_rez_h = atoi(argv[3]);
    }
  }
#endif

  UIState uistate;
  UIState *s = &uistate;
  ui_init(s);
  //BB init our UI
  bb_ui_init(s);
  
  pthread_t connect_thread_handle;
  err = pthread_create(&connect_thread_handle, NULL,
                       vision_connect_thread, s);
  assert(err == 0);

#ifdef QCOM
  pthread_t light_sensor_thread_handle;
  err = pthread_create(&light_sensor_thread_handle, NULL,
                       light_sensor_thread, s);
  assert(err == 0);

  pthread_t bg_thread_handle;
  err = pthread_create(&bg_thread_handle, NULL,
                       bg_thread, s);
  assert(err == 0);
#endif
  s->b.touch_last_width = s->scene.ui_viz_rw;
  TouchState touch = {0};
  touch_init(&touch);
  s->touch_fd = touch.fd;
  ui_sound_init();

  // light sensor scaling params
  const int LEON = is_leon();

  const float BRIGHTNESS_B = LEON ? 10.0 : 5.0;
  const float BRIGHTNESS_M = LEON ? 2.6 : 1.3;

  float smooth_brightness = BRIGHTNESS_B;

  const int MIN_VOLUME = LEON ? 12 : 9;
  const int MAX_VOLUME = LEON ? 15 : 12;

  set_volume(MIN_VOLUME);
  s->volume_timeout = 5 * UI_FREQ;
  int draws = 0;

  s->scene.satelliteCount = -1;

  while (!do_exit) {
    bool should_swap = false;
    if (!s->vision_connected) {
      // Delay a while to avoid 9% cpu usage while car is not started and user is keeping touching on the screen.
      // Don't hold the lock while sleeping, so that vision_connect_thread have chances to get the lock.
      usleep(30 * 1000);
    }
    pthread_mutex_lock(&s->lock);
    double u1 = millis_since_boot();

    // light sensor is only exposed on EONs
    float clipped_brightness = (s->light_sensor*BRIGHTNESS_M) + BRIGHTNESS_B;
    if (clipped_brightness > 512) clipped_brightness = 512;
    smooth_brightness = clipped_brightness * 0.01 + smooth_brightness * 0.99;
    if (smooth_brightness > 255) smooth_brightness = 255;
    set_brightness(s, (int)smooth_brightness);

    // resize vision for collapsing sidebar
    const bool hasSidebar = !s->scene.uilayout_sidebarcollapsed;
#if defined(QCOM) || defined(QCOM2)
    s->scene.ui_viz_rx = hasSidebar ? box_x : (box_x - sbr_w + (bdr_s * 2));
    s->scene.ui_viz_rw = hasSidebar ? box_w : (box_w + sbr_w - (bdr_s * 2));
    s->scene.ui_viz_ro = hasSidebar ? -(sbr_w - 6 * bdr_s) : 0;
#else
    s->scene.ui_viz_rx = hasSidebar ? box_x : (box_x - sbr_w);
    s->scene.ui_viz_rw = hasSidebar ? box_w : (box_w + sbr_w);
    s->scene.ui_viz_ro = hasSidebar ? -(sbr_w) : 0;
#endif
    // poll for touch events
    int touch_x = -1, touch_y = -1;
    int dc_touch_x = -1, dc_touch_y = -1;
    s->b.touch_timeout --;
    if (s->b.touch_timeout < 0) {
      s->b.touch_timeout = 0;
    }
    int touched = touch_poll(&touch, &touch_x, &touch_y, 0);
    if (touched == 1) {
      set_awake(s, true);
      handle_sidebar_touch(s, touch_x, touch_y);
      handle_vision_touch(s, touch_x, touch_y);
      s->b.touch_last = true;
      s->b.touch_last_x = touch_x;
      s->b.touch_last_y = touch_y;
      s->b.touch_timeout = touch_timeout;
    }
    //BB check touch
    if ((s->b.touch_last) && (s->b.touch_last_width != s->scene.ui_viz_rw)) {
      s->b.touch_last_width=s->scene.ui_viz_rw;
      bb_handle_ui_touch(s,s->b.touch_last_x,s->b.touch_last_y);
      dc_touch_x = s->b.touch_last_x;
      dc_touch_y = s->b.touch_last_y;
      s->b.touch_last = false;
      s->b.touch_last_x = 0;
      s->b.touch_last_y = 0;
    }

    
    //s->b.touch_last_width = s->scene.ui_viz_rw;
    //BB Update our cereal polls
    bb_ui_poll_update(s);

    if (!s->vision_connected) {
      if (s->status != STATUS_STOPPED) {
        update_status(s, STATUS_STOPPED);
      }
      check_messages(s);
    } else {
      set_awake(s, true);
      if (s->status == STATUS_STOPPED) {
        update_status(s, STATUS_DISENGAGED);
      }
      // Car started, fetch a new rgb image from ipc and peek for zmq events.
      ui_update(s);
      if (!s->vision_connected) {
        // Visiond process is just stopped, force a redraw to make screen blank again.
        s->scene.satelliteCount = -1;
        s->scene.uilayout_sidebarcollapsed = false;
        ui_draw(s);
        glFinish();
        should_swap = true;
      }
    }

    // manage wakefulness
    if (s->awake_timeout > 0) {
      s->awake_timeout--;
    } else {
      set_awake(s, false);
    }

    // manage hardware disconnect
    if (s->hardware_timeout > 0) {
      s->hardware_timeout--;
    } else {
      s->scene.hwType = cereal_HealthData_HwType_unknown;
    }

    // Don't waste resources on drawing in case screen is off
    if (s->awake) {
      ui_draw(s);
      if (s->vision_connected) {
	nvgScale(s->vg,s->b.scr_scale_x,s->b.scr_scale_y);
        bb_ui_draw_UI(s) ;
        ui_draw_infobar(s);
      }
      glFinish();
      should_swap = true;
    }

    if (s->volume_timeout > 0) {
      s->volume_timeout--;
    } else {
      int volume = fmin(MAX_VOLUME, MIN_VOLUME + s->scene.v_ego / 5);  // up one notch every 5 m/s
      set_volume(volume);
      s->volume_timeout = 5 * UI_FREQ;
    }

    if (s->controls_timeout > 0) {
      s->controls_timeout--;
    } else {
      // stop playing alert sound
      if ((!s->vision_connected || (s->vision_connected && s->alert_sound_timeout == 0)) &&
            s->alert_sound != cereal_CarControl_HUDControl_AudibleAlert_none) {
        stop_alert_sound(s->alert_sound);
        s->alert_sound = cereal_CarControl_HUDControl_AudibleAlert_none;
      }

      // if visiond is still running and controlsState times out, display an alert
      // TODO: refactor this to not be here
      if (s->controls_seen && s->vision_connected && strcmp(s->scene.alert_text2, "Controls Unresponsive") != 0) {
        s->scene.alert_size = ALERTSIZE_FULL;
        if (s->status != STATUS_STOPPED) {
          update_status(s, STATUS_ALERT);
        }
        snprintf(s->scene.alert_text1, sizeof(s->scene.alert_text1), "%s", "TAKE CONTROL IMMEDIATELY");
        snprintf(s->scene.alert_text2, sizeof(s->scene.alert_text2), "%s", "Controls Unresponsive");
        ui_draw_vision_alert(s, s->scene.alert_size, s->status, s->scene.alert_text1, s->scene.alert_text2);

        s->alert_sound_timeout = 2 * UI_FREQ;

        s->alert_sound = cereal_CarControl_HUDControl_AudibleAlert_chimeWarningRepeat;
        play_alert_sound(s->alert_sound);
      }
      s->alert_sound_timeout--;
      s->controls_seen = false;
    }

    read_param_bool_timeout(&s->is_metric, "IsMetric", &s->is_metric_timeout);
    read_param_bool_timeout(&s->longitudinal_control, "LongitudinalControl", &s->longitudinal_control_timeout);
    read_param_bool_timeout(&s->limit_set_speed, "LimitSetSpeed", &s->limit_set_speed_timeout);
    read_param_float_timeout(&s->speed_lim_off, "SpeedLimitOffset", &s->limit_set_speed_timeout);

    pthread_mutex_unlock(&s->lock);

    // the bg thread needs to be scheduled, so the main thread needs time without the lock
    // safe to do this outside the lock?
    if (should_swap) {
      double u2 = millis_since_boot();
      if (u2-u1 > 66) {
        // warn on sub 15fps
        LOGW("slow frame(%d) time: %.2f", draws, u2-u1);
      }
      draws++;
      framebuffer_swap(s->fb);
    }
  }

  set_awake(s, true);
  ui_sound_destroy();

  // wake up bg thread to exit
  pthread_mutex_lock(&s->lock);
  pthread_cond_signal(&s->bg_cond);
  pthread_mutex_unlock(&s->lock);

#ifdef QCOM
  // join light_sensor_thread?

  err = pthread_join(bg_thread_handle, NULL);
  assert(err == 0);
#endif

  err = pthread_join(connect_thread_handle, NULL);
  assert(err == 0);

  return 0;
}
