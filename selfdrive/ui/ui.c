#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/resource.h>

#include <cutils/properties.h>

#include <GLES3/gl3.h>
#include <EGL/egl.h>

#include <json.h>
#include <czmq.h>

#include "nanovg.h"
#define NANOVG_GLES3_IMPLEMENTATION
#include "nanovg_gl.h"
#include "nanovg_gl_utils.h"

#include "common/timing.h"
#include "common/util.h"
#include "common/swaglog.h"
#include "common/mat.h"
#include "common/glutil.h"

#include "common/touch.h"
#include "common/framebuffer.h"
#include "common/visionipc.h"
#include "common/visionimg.h"
#include "common/modeldata.h"
#include "common/params.h"

#include "cereal/gen/c/log.capnp.h"

// Calibration status values from controlsd.py
#define CALIBRATION_UNCALIBRATED 0
#define CALIBRATION_CALIBRATED 1
#define CALIBRATION_INVALID 2

#define STATUS_STOPPED 0
#define STATUS_DISENGAGED 1
#define STATUS_ENGAGED 2
#define STATUS_WARNING 3
#define STATUS_ALERT 4
#define STATUS_MAX 5

#define ALERTSIZE_NONE 0
#define ALERTSIZE_SMALL 1
#define ALERTSIZE_MID 2
#define ALERTSIZE_FULL 3

#define UI_BUF_COUNT 4

const int vwp_w = 1920;
const int vwp_h = 1080;
const int nav_w = 640;
const int nav_ww= 760;
const int sbr_w = 300;
const int bdr_s = 30;
const int box_x = sbr_w+bdr_s;
const int box_y = bdr_s;
const int box_w = vwp_w-sbr_w-(bdr_s*2);
const int box_h = vwp_h-(bdr_s*2);
const int viz_w = vwp_w-(bdr_s*2);
const int header_h = 420;

const uint8_t bg_colors[][4] = {
  [STATUS_STOPPED] = {0x07, 0x23, 0x39, 0xff},
  [STATUS_DISENGAGED] = {0x17, 0x33, 0x49, 0xff},
  [STATUS_ENGAGED] = {0x17, 0x86, 0x44, 0xff},
  [STATUS_WARNING] = {0xDA, 0x6F, 0x25, 0xff},
  [STATUS_ALERT] = {0xC9, 0x22, 0x31, 0xff},
};

const uint8_t alert_colors[][4] = {
  [STATUS_STOPPED] = {0x07, 0x23, 0x39, 0xf1},
  [STATUS_DISENGAGED] = {0x17, 0x33, 0x49, 0xc8},
  [STATUS_ENGAGED] = {0x17, 0x86, 0x44, 0xf1},
  [STATUS_WARNING] = {0xDA, 0x6F, 0x25, 0xf1},
  [STATUS_ALERT] = {0xC9, 0x22, 0x31, 0xf1},
};

const int alert_sizes[] = {
  [ALERTSIZE_NONE] = 0,
  [ALERTSIZE_SMALL] = 241,
  [ALERTSIZE_MID] = 390,
  [ALERTSIZE_FULL] = vwp_h,
};

typedef struct UIScene {
  int frontview;

  int transformed_width, transformed_height;

  uint64_t model_ts;
  ModelData model;

  float mpc_x[50];
  float mpc_y[50];

  bool world_objects_visible;
  mat3 warp_matrix;           // transformed box -> frame.
  mat4 extrinsic_matrix;      // Last row is 0 so we can use mat4.

  float v_cruise;
  uint64_t v_cruise_update_ts;
  float v_ego;
  float curvature;
  int engaged;
  bool engageable;

  bool uilayout_sidebarcollapsed;
  bool uilayout_mapenabled;
  // responsive layout
  int ui_viz_rx;
  int ui_viz_rw;
  int ui_viz_ro;

  int lead_status;
  float lead_d_rel, lead_y_rel, lead_v_rel;

  int front_box_x, front_box_y, front_box_width, front_box_height;

  uint64_t alert_ts;
  char alert_text1[1024];
  char alert_text2[1024];
  uint8_t alert_size;
  float alert_blinkingrate;

  float awareness_status;

  uint64_t started_ts;
  //BB CPU TEMP
  uint16_t maxCpuTemp;
  uint32_t maxBatTemp;
  float gpsAccuracy ;
  float freeSpace;
  float angleSteers;
  float angleSteersDes;
  //BB END CPU TEMP
  // Used to display calibration progress
  int cal_status;
  int cal_perc;
  // Used to show gps planner status
  bool gps_planner_active;

} UIScene;

typedef struct UIState {
  pthread_mutex_t lock;
  pthread_cond_t bg_cond;

  FramebufferState *fb;
  int fb_w, fb_h;
  EGLDisplay display;
  EGLSurface surface;

  NVGcontext *vg;
  //BB
  int custom_alert_playsound;
  //BB END
  int font_courbd;
  int font_sans_regular;
  int font_sans_semibold;
  int font_sans_bold;
  int img_wheel;

  zsock_t *thermal_sock;
  void *thermal_sock_raw;
  zsock_t *model_sock;
  void *model_sock_raw;
  zsock_t *live100_sock;
  void *live100_sock_raw;
  zsock_t *livecalibration_sock;
  void *livecalibration_sock_raw;
  zsock_t *live20_sock;
  void *live20_sock_raw;
  zsock_t *livempc_sock;
  void *livempc_sock_raw;
  zsock_t *plus_sock;
  void *plus_sock_raw;
  zsock_t *gps_sock;
  void *gps_sock_raw;

  zsock_t *uilayout_sock;
  void *uilayout_sock_raw;

  int plus_state;

  // vision state
  bool vision_connected;
  bool vision_connect_firstrun;
  int ipc_fd;

  VIPCBuf bufs[UI_BUF_COUNT];
  VIPCBuf front_bufs[UI_BUF_COUNT];
  int cur_vision_idx;
  int cur_vision_front_idx;

  GLuint frame_program;
  GLuint frame_texs[UI_BUF_COUNT];
  GLuint frame_front_texs[UI_BUF_COUNT];

  GLint frame_pos_loc, frame_texcoord_loc;
  GLint frame_texture_loc, frame_transform_loc;

  GLuint line_program;
  GLint line_pos_loc, line_color_loc;
  GLint line_transform_loc;

  unsigned int rgb_width, rgb_height, rgb_stride;
  size_t rgb_buf_len;
  mat4 rgb_transform;

  unsigned int rgb_front_width, rgb_front_height, rgb_front_stride;
  size_t rgb_front_buf_len;

  bool intrinsic_matrix_loaded;
  mat3 intrinsic_matrix;

  UIScene scene;

  bool awake;
  int awake_timeout;

  int status;
  bool is_metric;
  bool passive;
  int alert_size;
  float alert_blinking_alpha;
  bool alert_blinked;

  float light_sensor;
} UIState;

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
  if (awake) {
    // 30 second timeout at 30 fps
    s->awake_timeout = 30*30;
  }
  if (s->awake != awake) {
    s->awake = awake;

    if (awake) {
      LOG("awake normal");
      framebuffer_set_power(s->fb, HWC_POWER_MODE_NORMAL);
    } else {
      LOG("awake off");
      set_brightness(s, 0);
      framebuffer_set_power(s->fb, HWC_POWER_MODE_OFF);
    }
  }
}

volatile int do_exit = 0;
static void set_do_exit(int sig) {
  do_exit = 1;
}


static const char frame_vertex_shader[] =
  "attribute vec4 aPosition;\n"
  "attribute vec4 aTexCoord;\n"
  "uniform mat4 uTransform;\n"
  "varying vec4 vTexCoord;\n"
  "void main() {\n"
  "  gl_Position = uTransform * aPosition;\n"
  "  vTexCoord = aTexCoord;\n"
  "}\n";

static const char frame_fragment_shader[] =
  "precision mediump float;\n"
  "uniform sampler2D uTexture;\n"
  "varying vec4 vTexCoord;\n"
  "void main() {\n"
  "  gl_FragColor = texture2D(uTexture, vTexCoord.xy);\n"
  "}\n";

static const char line_vertex_shader[] =
  "attribute vec4 aPosition;\n"
  "attribute vec4 aColor;\n"
  "uniform mat4 uTransform;\n"
  "varying vec4 vColor;\n"
  "void main() {\n"
  "  gl_Position = uTransform * aPosition;\n"
  "  vColor = aColor;\n"
  "}\n";

static const char line_fragment_shader[] =
  "precision mediump float;\n"
  "uniform sampler2D uTexture;\n"
  "varying vec4 vColor;\n"
  "void main() {\n"
  "  gl_FragColor = vColor;\n"
  "}\n";


static const mat4 device_transform = {{
  1.0,  0.0, 0.0, 0.0,
  0.0,  1.0, 0.0, 0.0,
  0.0,  0.0, 1.0, 0.0,
  0.0,  0.0, 0.0, 1.0,
}};

// frame from 4/3 to box size with a 2x zoom
static const mat4 frame_transform = {{
  2*(4./3.)/((float)viz_w/box_h), 0.0, 0.0, 0.0,
                                           0.0, 2.0, 0.0, 0.0,
                                           0.0, 0.0, 1.0, 0.0,
                                           0.0, 0.0, 0.0, 1.0,
}};

static void ui_init(UIState *s) {
  memset(s, 0, sizeof(UIState));

  pthread_mutex_init(&s->lock, NULL);
  pthread_cond_init(&s->bg_cond, NULL);

  // init connections

  s->thermal_sock = zsock_new_sub(">tcp://127.0.0.1:8005", "");
  assert(s->thermal_sock);
  s->thermal_sock_raw = zsock_resolve(s->thermal_sock);

  s->gps_sock = zsock_new_sub(">tcp://127.0.0.1:8032", "");
  assert(s->gps_sock);
  s->gps_sock_raw = zsock_resolve(s->gps_sock);
  
  s->model_sock = zsock_new_sub(">tcp://127.0.0.1:8009", "");
  assert(s->model_sock);
  s->model_sock_raw = zsock_resolve(s->model_sock);

  s->live100_sock = zsock_new_sub(">tcp://127.0.0.1:8007", "");
  assert(s->live100_sock);
  s->live100_sock_raw = zsock_resolve(s->live100_sock);

  s->uilayout_sock = zsock_new_sub(">tcp://127.0.0.1:8060", "");
  assert(s->uilayout_sock);
  s->uilayout_sock_raw = zsock_resolve(s->uilayout_sock);

  s->livecalibration_sock = zsock_new_sub(">tcp://127.0.0.1:8019", "");
  assert(s->livecalibration_sock);
  s->livecalibration_sock_raw = zsock_resolve(s->livecalibration_sock);

  s->live20_sock = zsock_new_sub(">tcp://127.0.0.1:8012", "");
  assert(s->live20_sock);
  s->live20_sock_raw = zsock_resolve(s->live20_sock);

  s->livempc_sock = zsock_new_sub(">tcp://127.0.0.1:8035", "");
  assert(s->livempc_sock);
  s->livempc_sock_raw = zsock_resolve(s->livempc_sock);

  s->plus_sock = zsock_new_sub(">tcp://127.0.0.1:8037", "");
  assert(s->plus_sock);
  s->plus_sock_raw = zsock_resolve(s->plus_sock);

  s->ipc_fd = -1;
  s->custom_alert_playsound=0;

  // init display
  s->fb = framebuffer_init("ui", 0x00010000, true,
                           &s->display, &s->surface, &s->fb_w, &s->fb_h);
  assert(s->fb);

  set_awake(s, true);

  // init drawing
  s->vg = nvgCreateGLES3(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  assert(s->vg);

  s->font_courbd = nvgCreateFont(s->vg, "courbd", "../assets/courbd.ttf");
  assert(s->font_courbd >= 0);
  s->font_sans_regular = nvgCreateFont(s->vg, "sans-regular", "../assets/OpenSans-Regular.ttf");
  assert(s->font_sans_regular >= 0);
  s->font_sans_semibold = nvgCreateFont(s->vg, "sans-semibold", "../assets/OpenSans-SemiBold.ttf");
  assert(s->font_sans_semibold >= 0);
  s->font_sans_bold = nvgCreateFont(s->vg, "sans-bold", "../assets/OpenSans-Bold.ttf");
  assert(s->font_sans_bold >= 0);

  assert(s->img_wheel >= 0);
  s->img_wheel = nvgCreateImage(s->vg, "../assets/img_chffr_wheel.png", 1);

  // init gl
  s->frame_program = load_program(frame_vertex_shader, frame_fragment_shader);
  assert(s->frame_program);

  s->frame_pos_loc = glGetAttribLocation(s->frame_program, "aPosition");
  s->frame_texcoord_loc = glGetAttribLocation(s->frame_program, "aTexCoord");

  s->frame_texture_loc = glGetUniformLocation(s->frame_program, "uTexture");
  s->frame_transform_loc = glGetUniformLocation(s->frame_program, "uTransform");

  s->line_program = load_program(line_vertex_shader, line_fragment_shader);
  assert(s->line_program);

  s->line_pos_loc = glGetAttribLocation(s->line_program, "aPosition");
  s->line_color_loc = glGetAttribLocation(s->line_program, "aColor");
  s->line_transform_loc = glGetUniformLocation(s->line_program, "uTransform");

  glViewport(0, 0, s->fb_w, s->fb_h);

  glDisable(GL_DEPTH_TEST);

  assert(glGetError() == GL_NO_ERROR);

  {
    char *value;
    const int result = read_db_value(NULL, "Passive", &value, NULL);
    if (result == 0) {
      s->passive = value[0] == '1';
      free(value);
    }
  }
}

// If the intrinsics are in the params entry, this copies them to
// intrinsic_matrix and returns true.  Otherwise returns false.
static bool try_load_intrinsics(mat3 *intrinsic_matrix) {
  char *value;
  const int result = read_db_value(NULL, "CloudCalibration", &value, NULL);

  if (result == 0) {
    JsonNode* calibration_json = json_decode(value);
    free(value);

    JsonNode *intrinsic_json =
        json_find_member(calibration_json, "intrinsic_matrix");

    if (intrinsic_json == NULL || intrinsic_json->tag != JSON_ARRAY) {
      json_delete(calibration_json);
      return false;
    }

    int i = 0;
    JsonNode* json_num;
    json_foreach(json_num, intrinsic_json) {
      intrinsic_matrix->v[i++] = json_num->number_;
    }
    json_delete(calibration_json);

    return true;
  } else {
    return false;
  }
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
      .cal_status = CALIBRATION_CALIBRATED,
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
    2.0/s->rgb_width, 0.0, 0.0, -1.0,
    0.0, 2.0/s->rgb_height, 0.0, -1.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
  }};

  char *value;
  const int result = read_db_value(NULL, "IsMetric", &value, NULL);
  if (result == 0) {
    s->is_metric = value[0] == '1';
    free(value);
  }
}

static void ui_draw_transformed_box(UIState *s, uint32_t color) {
  const UIScene *scene = &s->scene;

  const mat3 bbt = scene->warp_matrix;

  struct {
    vec3 pos;
    uint32_t color;
  } verts[] = {
    {matvecmul3(bbt, (vec3){{0.0, 0.0, 1.0,}}), color},
    {matvecmul3(bbt, (vec3){{scene->transformed_width, 0.0, 1.0,}}), color},
    {matvecmul3(bbt, (vec3){{scene->transformed_width, scene->transformed_height, 1.0,}}), color},
    {matvecmul3(bbt, (vec3){{0.0, scene->transformed_height, 1.0,}}), color},
    {matvecmul3(bbt, (vec3){{0.0, 0.0, 1.0,}}), color},
  };

  for (int i=0; i<ARRAYSIZE(verts); i++) {
    verts[i].pos.v[0] = verts[i].pos.v[0] / verts[i].pos.v[2];
    verts[i].pos.v[1] = s->rgb_height - verts[i].pos.v[1] / verts[i].pos.v[2];
  }

  glUseProgram(s->line_program);

  mat4 out_mat = matmul(device_transform,
                        matmul(frame_transform, s->rgb_transform));
  glUniformMatrix4fv(s->line_transform_loc, 1, GL_TRUE, out_mat.v);

  glEnableVertexAttribArray(s->line_pos_loc);
  glVertexAttribPointer(s->line_pos_loc, 2, GL_FLOAT, GL_FALSE, sizeof(verts[0]), &verts[0].pos.v[0]);

  glEnableVertexAttribArray(s->line_color_loc);
  glVertexAttribPointer(s->line_color_loc, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(verts[0]), &verts[0].color);

  assert(glGetError() == GL_NO_ERROR);
  glDrawArrays(GL_LINE_STRIP, 0, ARRAYSIZE(verts));
}

// Projects a point in car to space to the corresponding point in full frame
// image space.
vec3 car_space_to_full_frame(const UIState *s, vec4 car_space_projective) {
  const UIScene *scene = &s->scene;

  // We'll call the car space point p.
  // First project into normalized image coordinates with the extrinsics matrix.
  const vec4 Ep4 = matvecmul(scene->extrinsic_matrix, car_space_projective);

  // The last entry is zero because of how we store E (to use matvecmul).
  const vec3 Ep = {{Ep4.v[0], Ep4.v[1], Ep4.v[2]}};
  const vec3 KEp = matvecmul3(s->intrinsic_matrix, Ep);

  // Project.
  const vec3 p_image = {{KEp.v[0] / KEp.v[2], KEp.v[1] / KEp.v[2], 1.}};
  return p_image;
}

// Calculate an interpolation between two numbers at a specific increment
static float lerp(float v0, float v1, float t) {
  return (1 - t) * v0 + t * v1;
}

static void draw_chevron(UIState *s, float x_in, float y_in, float sz,
                          NVGcolor fillColor, NVGcolor glowColor) {
  const UIScene *scene = &s->scene;

  nvgSave(s->vg);

  nvgTranslate(s->vg, 240.0f, 0.0);
  nvgTranslate(s->vg, -1440.0f / 2, -1080.0f / 2);
  nvgScale(s->vg, 2.0, 2.0);
  nvgScale(s->vg, 1440.0f / s->rgb_width, 1080.0f / s->rgb_height);

  const vec4 p_car_space = (vec4){{x_in, y_in, 0., 1.}};
  const vec3 p_full_frame = car_space_to_full_frame(s, p_car_space);

  sz *= 30;
  sz /= (x_in / 3 + 30);
  if (sz > 30) sz = 30;
  if (sz < 15) sz = 15;

  float x = p_full_frame.v[0];
  float y = p_full_frame.v[1];

  // glow
  nvgBeginPath(s->vg);
  float g_xo = sz/5;
  float g_yo = sz/10;
  if (x >= 0 && y >= 0.) {
    nvgMoveTo(s->vg, x+(sz*1.35)+g_xo, y+sz+g_yo);
    nvgLineTo(s->vg, x, y-g_xo);
    nvgLineTo(s->vg, x-(sz*1.35)-g_xo, y+sz+g_yo);
    nvgLineTo(s->vg, x+(sz*1.35)+g_xo, y+sz+g_yo);
    nvgClosePath(s->vg);
  }
  nvgFillColor(s->vg, glowColor);
  nvgFill(s->vg);

  // chevron
  nvgBeginPath(s->vg);
  if (x >= 0 && y >= 0.) {
    nvgMoveTo(s->vg, x+(sz*1.25), y+sz);
    nvgLineTo(s->vg, x, y);
    nvgLineTo(s->vg, x-(sz*1.25), y+sz);
    nvgLineTo(s->vg, x+(sz*1.25), y+sz);
    nvgClosePath(s->vg);
  }
  nvgFillColor(s->vg, fillColor);
  nvgFill(s->vg);

  nvgRestore(s->vg);
}

static void ui_draw_lane_line(UIState *s, const float *points, float off,
                      NVGcolor color, bool is_ghost) {
  const UIScene *scene = &s->scene;

  nvgSave(s->vg);
  nvgTranslate(s->vg, 240.0f, 0.0); // rgb-box space
  nvgTranslate(s->vg, -1440.0f / 2, -1080.0f / 2); // zoom 2x
  nvgScale(s->vg, 2.0, 2.0);
  nvgScale(s->vg, 1440.0f / s->rgb_width, 1080.0f / s->rgb_height);
  nvgBeginPath(s->vg);

  bool started = false;
  for (int i=0; i<49; i++) {
    float px = (float)i;
    float py = points[i] - off;
    vec4 p_car_space = (vec4){{px, py, 0., 1.}};
    vec3 p_full_frame = car_space_to_full_frame(s, p_car_space);
    float x = p_full_frame.v[0];
    float y = p_full_frame.v[1];
    if (x < 0 || y < 0.) {
      continue;
    }
    if (!started) {
      nvgMoveTo(s->vg, x, y);
      started = true;
    } else {
      nvgLineTo(s->vg, x, y);
    }
  }

  for (int i=49; i>0; i--) {
    float px = (float)i;
    float py = is_ghost?(points[i]-off):(points[i]+off);
    vec4 p_car_space = (vec4){{px, py, 0., 1.}};
    vec3 p_full_frame = car_space_to_full_frame(s, p_car_space);
    float x = p_full_frame.v[0];
    float y = p_full_frame.v[1];
    if (x < 0 || y < 0.) {
      continue;
    }
    nvgLineTo(s->vg, x, y);
  }

  nvgClosePath(s->vg);
  nvgFillColor(s->vg, color);
  nvgFill(s->vg);
  nvgRestore(s->vg);
}

static void ui_draw_lane(UIState *s, const PathData path, NVGcolor color) {
  ui_draw_lane_line(s, path.points, 0.025*path.prob, color, false);
  float var = min(path.std, 0.7);
  color.a /= 4;
  ui_draw_lane_line(s, path.points, -var, color, true);
  ui_draw_lane_line(s, path.points, var, color, true);
}

static void ui_draw_track(UIState *s, bool is_mpc) {
  const UIScene *scene = &s->scene;
  const PathData path = scene->model.path;
  const float *mpc_x_coords = &scene->mpc_x[0];
  const float *mpc_y_coords = &scene->mpc_y[0];

  nvgSave(s->vg);
  nvgTranslate(s->vg, 240.0f, 0.0); // rgb-box space
  nvgTranslate(s->vg, -1440.0f / 2, -1080.0f / 2); // zoom 2x
  nvgScale(s->vg, 2.0, 2.0);
  nvgScale(s->vg, 1440.0f / s->rgb_width, 1080.0f / s->rgb_height);
  nvgBeginPath(s->vg);

  bool started = false;
  float off = is_mpc?0.3:0.5;
  float lead_d = scene->lead_d_rel*2.;
  float path_height = is_mpc?(lead_d>5.)?min(lead_d, 25.)-min(lead_d*0.35, 10.):20.
                            :(lead_d>0.)?min(lead_d, 50.)-min(lead_d*0.35, 10.):49.;

  // left side up
  for (int i=0; i<=path_height; i++) {
    float px, py, mpx;
    if (is_mpc) {
      mpx = i==0?0.0:mpc_x_coords[i];
      px = lerp(mpx+1.0, mpx, i/100.0);
      py = mpc_y_coords[i] - off;
    } else {
      px = lerp(i+1.0, i, i/100.0);
      py = path.points[i] - off;
    }

    vec4 p_car_space = (vec4){{px, py, 0., 1.}};
    vec3 p_full_frame = car_space_to_full_frame(s, p_car_space);
    float x = p_full_frame.v[0];
    float y = p_full_frame.v[1];
    if (x < 0 || y < 0) {
      continue;
    }

    if (!started) {
      nvgMoveTo(s->vg, x, y);
      started = true;
    } else {
      nvgLineTo(s->vg, x, y);
    }
  }

  // right side down
  for (int i=path_height; i>=0; i--) {
    float px, py, mpx;
    if (is_mpc) {
      mpx = i==0?0.0:mpc_x_coords[i];
      px = lerp(mpx+1.0, mpx, i/100.0);
      py = mpc_y_coords[i] + off;
    } else {
      px = lerp(i+1.0, i, i/100.0);
      py = path.points[i] + off;
    }

    vec4 p_car_space = (vec4){{px, py, 0., 1.}};
    vec3 p_full_frame = car_space_to_full_frame(s, p_car_space);
    float x = p_full_frame.v[0];
    float y = p_full_frame.v[1];
    if (x < 0 || y < 0.) {
      continue;
    }

    nvgLineTo(s->vg, x, y);
  }

  nvgClosePath(s->vg);

  NVGpaint track_bg;
  if (is_mpc) {
    // Draw colored MPC track
    const uint8_t *clr = bg_colors[s->status];
    track_bg = nvgLinearGradient(s->vg, vwp_w, vwp_h, vwp_w, vwp_h*.4,
      nvgRGBA(clr[0], clr[1], clr[2], 255), nvgRGBA(clr[0], clr[1], clr[2], 255/2));
  } else {
    // Draw white vision track
    track_bg = nvgLinearGradient(s->vg, vwp_w, vwp_h, vwp_w, vwp_h*.4,
      nvgRGBA(255, 255, 255, 255), nvgRGBA(255, 255, 255, 0));
  }

  nvgFillPaint(s->vg, track_bg);
  nvgFill(s->vg);
  nvgRestore(s->vg);
}

static void draw_steering(UIState *s, float curvature) {
  float points[50];
  for (int i = 0; i < 50; i++) {
    float y_actual = i * tan(asin(clamp(i * curvature, -0.999, 0.999)) / 2.);
    points[i] = y_actual;
  }

  // ui_draw_lane_edge(s, points, 0.0, nvgRGBA(0, 0, 255, 128), 5);
}

static void draw_frame(UIState *s) {
  const UIScene *scene = &s->scene;

  mat4 out_mat;
  float x1, x2, y1, y2;
  if (s->scene.frontview) {
    out_mat = device_transform; // full 16/9
    // flip horizontally so it looks like a mirror
    x1 = (float)scene->front_box_x / s->rgb_front_width;
    x2 = (float)(scene->front_box_x + scene->front_box_width) / s->rgb_front_width;
    y2 = (float)scene->front_box_y / s->rgb_front_height;
    y1 = (float)(scene->front_box_y + scene->front_box_height) / s->rgb_front_height;
  } else {
    out_mat = matmul(device_transform, frame_transform);
    x1 = 1.0;
    x2 = 0.0;
    y1 = 1.0;
    y2 = 0.0;
  }

  const uint8_t frame_indicies[] = {0, 1, 2, 0, 2, 3};
  const float frame_coords[4][4] = {
    {-1.0, -1.0, x2, y1}, //bl
    {-1.0,  1.0, x2, y2}, //tl
    { 1.0,  1.0, x1, y2}, //tr
    { 1.0, -1.0, x1, y1}, //br
  };

  glActiveTexture(GL_TEXTURE0);
  if (s->scene.frontview && s->cur_vision_front_idx >= 0) {
    glBindTexture(GL_TEXTURE_2D, s->frame_front_texs[s->cur_vision_front_idx]);
  } else if (!scene->frontview && s->cur_vision_idx >= 0) {
    glBindTexture(GL_TEXTURE_2D, s->frame_texs[s->cur_vision_idx]);
  }

  glUseProgram(s->frame_program);

  glUniform1i(s->frame_texture_loc, 0);
  glUniformMatrix4fv(s->frame_transform_loc, 1, GL_TRUE, out_mat.v);

  glEnableVertexAttribArray(s->frame_pos_loc);
  glVertexAttribPointer(s->frame_pos_loc, 2, GL_FLOAT, GL_FALSE,
                        sizeof(frame_coords[0]), frame_coords);

  glEnableVertexAttribArray(s->frame_texcoord_loc);
  glVertexAttribPointer(s->frame_texcoord_loc, 2, GL_FLOAT, GL_FALSE,
                        sizeof(frame_coords[0]), &frame_coords[0][2]);

  assert(glGetError() == GL_NO_ERROR);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, &frame_indicies[0]);
}

static void ui_draw_vision_lanes(UIState *s) {
  const UIScene *scene = &s->scene;
  // Draw left lane edge
  ui_draw_lane(
      s, scene->model.left_lane,
      nvgRGBAf(1.0, 1.0, 1.0, scene->model.left_lane.prob));

  // Draw right lane edge
  ui_draw_lane(
      s, scene->model.right_lane,
      nvgRGBAf(1.0, 1.0, 1.0, scene->model.right_lane.prob));

  // Draw vision path
  ui_draw_track(s, false);

  if (scene->engaged) {
    // Draw MPC path when engaged
    ui_draw_track(s, true);
  }
}

// Draw all world space objects.
static void ui_draw_world(UIState *s) {
  const UIScene *scene = &s->scene;
  if (!scene->world_objects_visible) {
    return;
  }

  if ((nanos_since_boot() - scene->model_ts) < 1000000000ULL) {
    // Draw lane edges and vision/mpc tracks
    ui_draw_vision_lanes(s);
  }

  if (scene->lead_status) {
    // Draw lead car indicator
    float fillAlpha = 0;
    float speedBuff = 10.;
    float leadBuff = 40.;
    if (scene->lead_d_rel < leadBuff) {
      fillAlpha = 255*(1.0-(scene->lead_d_rel/leadBuff));
      if (scene->lead_v_rel < 0) {
        fillAlpha += 255*(-1*(scene->lead_v_rel/speedBuff));
      }
      fillAlpha = (int)(min(fillAlpha, 255));
    }
    draw_chevron(s, scene->lead_d_rel+2.7, scene->lead_y_rel, 25,
                  nvgRGBA(201, 34, 49, fillAlpha), nvgRGBA(218, 202, 37, 255));
  }
}

static void update_status(UIState *s, int status) {
  if (s->status != status) {
    s->status = status;
    // wake up bg thread to change
    pthread_cond_signal(&s->bg_cond);
    //if tesla call the sound command
    if ((status ==3 ) && (s->custom_alert_playsound > 0)) {
      return;
    }
    char* snd_command;
    if ((status ==3 ) && (s->custom_alert_playsound == 1)) {
      s->custom_alert_playsound = 2;
    }
    asprintf(&snd_command, "python /data/openpilot/selfdrive/car/tesla/snd/playsound.py %d &", status);
    system(snd_command);
  }
}

//BB START: functions added for the display of various items
static int bb_ui_draw_measure(UIState *s,  const char* bb_value, const char* bb_uom, const char* bb_label, 
		int bb_x, int bb_y, int bb_uom_dx,
		NVGcolor bb_valueColor, NVGcolor bb_labelColor, NVGcolor bb_uomColor, 
		int bb_valueFontSize, int bb_labelFontSize, int bb_uomFontSize )  {
  const UIScene *scene = &s->scene;	
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  int dx = 0;
  if (strlen(bb_uom) > 0) {
  	dx = (int)(bb_uomFontSize*2.5/2);
   }
  //print value
  nvgFontFace(s->vg, "sans-semibold");
  nvgFontSize(s->vg, bb_valueFontSize*2.5);
  nvgFillColor(s->vg, bb_valueColor);
  nvgText(s->vg, bb_x-dx/2, bb_y+ (int)(bb_valueFontSize*2.5)+5, bb_value, NULL);
  //print label
  nvgFontFace(s->vg, "sans-regular");
  nvgFontSize(s->vg, bb_labelFontSize*2.5);
  nvgFillColor(s->vg, bb_labelColor);
  nvgText(s->vg, bb_x, bb_y + (int)(bb_valueFontSize*2.5)+5 + (int)(bb_labelFontSize*2.5)+5, bb_label, NULL);
  //print uom
  if (strlen(bb_uom) > 0) {
      nvgSave(s->vg);
	  int rx =bb_x + bb_uom_dx + bb_valueFontSize -3;
	  int ry = bb_y + (int)(bb_valueFontSize*2.5/2)+25;
	  nvgTranslate(s->vg,rx,ry);
	  nvgRotate(s->vg, -1.5708); //-90deg in radians
	  nvgFontFace(s->vg, "sans-regular");
	  nvgFontSize(s->vg, (int)(bb_uomFontSize*2.5));
	  nvgFillColor(s->vg, bb_uomColor);
	  nvgText(s->vg, 0, 0, bb_uom, NULL);
	  nvgRestore(s->vg);
  }
  return (int)((bb_valueFontSize + bb_labelFontSize)*2.5) + 5;
}

static void bb_ui_draw_custom_alert(UIState *s,char *car_model) {
  const UIScene *scene = &s->scene;
  char *filepath = malloc(90);
  sprintf(filepath,"/data/openpilot/selfdrive/car/%s/alert.msg",car_model);
  //get 3-state switch position
  int alert_msg_fd;
  char alert_msg[1000];
  if (strlen(s->scene.alert_text1) > 0) {
    //already a system alert, ignore ours
    return;
  }
  alert_msg_fd = open (filepath, O_RDONLY);
  //if we can't open then done
  if (alert_msg_fd == -1) {
    s->custom_alert_playsound = 0;
    return;
  } else {
    int rd = read(alert_msg_fd, &(s->scene.alert_text1), 1000);
    if ((rd > 1) && (s->scene.alert_text1[rd-1] == '^')) {
      //^ as last character means warning
      if (s->custom_alert_playsound==0) {
        //sound never played, request sound
        s->custom_alert_playsound=1;
      }
      //update_status(s,3); //ALERT_WARNINGa
      s->status = 3; //ALERT_WARNING
      rd --;
    }
    s->scene.alert_text1[rd] = '\0';
    close(alert_msg_fd);
    if (strlen(s->scene.alert_text1) > 0) {
      s->scene.alert_size = ALERTSIZE_SMALL;
    } else {
      s->scene.alert_size = ALERTSIZE_NONE;
      s->scene.alert_text1[0]=0;
      s->custom_alert_playsound = 0;
    }
  }
}

static void bb_ui_draw_measures_left(UIState *s, int bb_x, int bb_y, int bb_w ) {
	const UIScene *scene = &s->scene;		
	int bb_rx = bb_x + (int)(bb_w/2);
	int bb_ry = bb_y;
	int bb_h = 5; 
	NVGcolor lab_color = nvgRGBA(255, 255, 255, 200);
	NVGcolor uom_color = nvgRGBA(255, 255, 255, 200);
	int value_fontSize=30;
	int label_fontSize=15;
	int uom_fontSize = 15;
	int bb_uom_dx =  (int)(bb_w /2 - uom_fontSize*2.5) ;
	
	//add CPU temperature
	if (true) {
	    	char val_str[16];
		char uom_str[6];
		NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
			if((int)(scene->maxCpuTemp/10) > 80) {
				val_color = nvgRGBA(255, 188, 3, 200);
			}
			if((int)(scene->maxCpuTemp/10) > 92) {
				val_color = nvgRGBA(255, 0, 0, 200);
			}
			// temp is alway in C * 10
			if (s->is_metric) {
				 snprintf(val_str, sizeof(val_str), "%d C", (int)(scene->maxCpuTemp/10));
			} else {
				 snprintf(val_str, sizeof(val_str), "%d F", (int)(32+9*(scene->maxCpuTemp/10)/5));
			}
		snprintf(uom_str, sizeof(uom_str), "");
		bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "CPU TEMP", 
				bb_rx, bb_ry, bb_uom_dx,
				val_color, lab_color, uom_color, 
				value_fontSize, label_fontSize, uom_fontSize );
		bb_ry = bb_y + bb_h;
	}

   //add battery temperature
	if (true) {
		char val_str[16];
		char uom_str[6];
		NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
		if((int)(scene->maxBatTemp/1000) > 40) {
			val_color = nvgRGBA(255, 188, 3, 200);
		}
		if((int)(scene->maxBatTemp/1000) > 50) {
			val_color = nvgRGBA(255, 0, 0, 200);
		}
		// temp is alway in C * 1000
		if (s->is_metric) {
			 snprintf(val_str, sizeof(val_str), "%d C", (int)(scene->maxBatTemp/1000));
		} else {
			 snprintf(val_str, sizeof(val_str), "%d F", (int)(32+9*(scene->maxBatTemp/1000)/5));
		}
		snprintf(uom_str, sizeof(uom_str), "");
		bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "BAT TEMP", 
				bb_rx, bb_ry, bb_uom_dx,
				val_color, lab_color, uom_color, 
				value_fontSize, label_fontSize, uom_fontSize );
		bb_ry = bb_y + bb_h;
	}
	
	//add grey panda GPS accuracy
	if (true) {
		char val_str[16];
		char uom_str[3];
		NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
		//show red/orange if gps accuracy is high
	    if(scene->gpsAccuracy > 0.59) {
	       val_color = nvgRGBA(255, 188, 3, 200);
	    }
	    if(scene->gpsAccuracy > 0.8) {
	       val_color = nvgRGBA(255, 0, 0, 200);
	    }


		// gps accuracy is always in meters
		if (s->is_metric) {
			 snprintf(val_str, sizeof(val_str), "%d", (int)(s->scene.gpsAccuracy*100.0));
		} else {
			 snprintf(val_str, sizeof(val_str), "%.1f", s->scene.gpsAccuracy * 3.28084 * 12);
		}
		if (s->is_metric) {
			snprintf(uom_str, sizeof(uom_str), "cm");;
		} else {
			snprintf(uom_str, sizeof(uom_str), "in");
		}
		bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "GPS PREC", 
				bb_rx, bb_ry, bb_uom_dx,
				val_color, lab_color, uom_color, 
				value_fontSize, label_fontSize, uom_fontSize );
		bb_ry = bb_y + bb_h;
	}
  //add free space - from bthaler1
	if (true) {
		char val_str[16];
		char uom_str[3];
		NVGcolor val_color = nvgRGBA(255, 255, 255, 200);

		//show red/orange if free space is low
		if(scene->freeSpace < 0.4) {
			val_color = nvgRGBA(255, 188, 3, 200);
		}
		if(scene->freeSpace < 0.2) {
			val_color = nvgRGBA(255, 0, 0, 200);
		}

		snprintf(val_str, sizeof(val_str), "%.1f", s->scene.freeSpace* 100);
		snprintf(uom_str, sizeof(uom_str), "%%");

		bb_h +=bb_ui_draw_measure(s, val_str, uom_str, "FREE", 
			bb_rx, bb_ry, bb_uom_dx,
			val_color, lab_color, uom_color, 
			value_fontSize, label_fontSize, uom_fontSize );
		bb_ry = bb_y + bb_h;
	}	
	//finally draw the frame
	bb_h += 20;
	nvgBeginPath(s->vg);
  	nvgRoundedRect(s->vg, bb_x, bb_y, bb_w, bb_h, 20);
  	nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
  	nvgStrokeWidth(s->vg, 6);
  	nvgStroke(s->vg);
}


static void bb_ui_draw_measures_right(UIState *s, int bb_x, int bb_y, int bb_w ) {
	const UIScene *scene = &s->scene;		
	int bb_rx = bb_x + (int)(bb_w/2);
	int bb_ry = bb_y;
	int bb_h = 5; 
	NVGcolor lab_color = nvgRGBA(255, 255, 255, 200);
	NVGcolor uom_color = nvgRGBA(255, 255, 255, 200);
	int value_fontSize=30;
	int label_fontSize=15;
	int uom_fontSize = 15;
	int bb_uom_dx =  (int)(bb_w /2 - uom_fontSize*2.5) ;
	
	//add visual radar relative distance
	if (true) {
		char val_str[16];
		char uom_str[6];
		NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
		if (scene->lead_status) {
			//show RED if less than 5 meters
			//show orange if less than 15 meters
			if((int)(scene->lead_d_rel) < 15) {
				val_color = nvgRGBA(255, 188, 3, 200);
			}
			if((int)(scene->lead_d_rel) < 5) {
				val_color = nvgRGBA(255, 0, 0, 200);
			}
			// lead car relative distance is always in meters
			if (s->is_metric) {
				 snprintf(val_str, sizeof(val_str), "%d", (int)scene->lead_d_rel);
			} else {
				 snprintf(val_str, sizeof(val_str), "%d", (int)(scene->lead_d_rel * 3.28084));
			}
		} else {
		   snprintf(val_str, sizeof(val_str), "-");
		}
		if (s->is_metric) {
			snprintf(uom_str, sizeof(uom_str), "m   ");
		} else {
			snprintf(uom_str, sizeof(uom_str), "ft");
		}
		bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "REL DIST", 
				bb_rx, bb_ry, bb_uom_dx,
				val_color, lab_color, uom_color, 
				value_fontSize, label_fontSize, uom_fontSize );
		bb_ry = bb_y + bb_h;
	}
	
	//add visual radar relative speed
	if (true) {
		char val_str[16];
		char uom_str[6];
		NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
		if (scene->lead_status) {
			//show Orange if negative speed (approaching)
			//show Orange if negative speed faster than 5mph (approaching fast)
			if((int)(scene->lead_v_rel) < 0) {
				val_color = nvgRGBA(255, 188, 3, 200);
			}
			if((int)(scene->lead_v_rel) < -5) {
				val_color = nvgRGBA(255, 0, 0, 200);
			}
			// lead car relative speed is always in meters
			if (s->is_metric) {
				 snprintf(val_str, sizeof(val_str), "%d", (int)(scene->lead_v_rel * 3.6 + 0.5));
			} else {
				 snprintf(val_str, sizeof(val_str), "%d", (int)(scene->lead_v_rel * 2.2374144 + 0.5));
			}
		} else {
		   snprintf(val_str, sizeof(val_str), "-");
		}
		if (s->is_metric) {
			snprintf(uom_str, sizeof(uom_str), "km/h");;
		} else {
			snprintf(uom_str, sizeof(uom_str), "mph");
		}
		bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "REL SPD", 
				bb_rx, bb_ry, bb_uom_dx,
				val_color, lab_color, uom_color, 
				value_fontSize, label_fontSize, uom_fontSize );
		bb_ry = bb_y + bb_h;
	}
	
	//add  steering angle
	if (true) {
		char val_str[16];
		char uom_str[6];
		NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
			//show Orange if more than 6 degrees
			//show red if  more than 12 degrees
			if(((int)(scene->angleSteers) < -6) || ((int)(scene->angleSteers) > 6)) {
				val_color = nvgRGBA(255, 188, 3, 200);
			}
			if(((int)(scene->angleSteers) < -12) || ((int)(scene->angleSteers) > 12)) {
				val_color = nvgRGBA(255, 0, 0, 200);
			}
			// steering is in degrees
			snprintf(val_str, sizeof(val_str), "%.1f",(scene->angleSteers));

	    snprintf(uom_str, sizeof(uom_str), "deg");
		bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "STEER", 
				bb_rx, bb_ry, bb_uom_dx,
				val_color, lab_color, uom_color, 
				value_fontSize, label_fontSize, uom_fontSize );
		bb_ry = bb_y + bb_h;
	}
	
	//add  desired steering angle
	if (true) {
		char val_str[16];
		char uom_str[6];
		NVGcolor val_color = nvgRGBA(255, 255, 255, 200);
			//show Orange if more than 6 degrees
			//show red if  more than 12 degrees
			if(((int)(scene->angleSteersDes) < -6) || ((int)(scene->angleSteersDes) > 6)) {
				val_color = nvgRGBA(255, 188, 3, 200);
			}
			if(((int)(scene->angleSteersDes) < -12) || ((int)(scene->angleSteersDes) > 12)) {
				val_color = nvgRGBA(255, 0, 0, 200);
			}
			// steering is in degrees
			snprintf(val_str, sizeof(val_str), "%.1f",(scene->angleSteersDes));

	    snprintf(uom_str, sizeof(uom_str), "deg");
		bb_h +=bb_ui_draw_measure(s,  val_str, uom_str, "DES STEER", 
				bb_rx, bb_ry, bb_uom_dx,
				val_color, lab_color, uom_color, 
				value_fontSize, label_fontSize, uom_fontSize );
		bb_ry = bb_y + bb_h;
	}
	
	
	//finally draw the frame
	bb_h += 20;
	nvgBeginPath(s->vg);
  	nvgRoundedRect(s->vg, bb_x, bb_y, bb_w, bb_h, 20);
  	nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
  	nvgStrokeWidth(s->vg, 6);
  	nvgStroke(s->vg);
}

//draw grid from wiki
static void ui_draw_vision_grid(UIState *s) {
  const UIScene *scene = &s->scene;
  bool is_cruise_set = (s->scene.v_cruise != 0 && s->scene.v_cruise != 255);
  if (!is_cruise_set) {
    const int grid_spacing = 30;

    int ui_viz_rx = scene->ui_viz_rx;
    int ui_viz_rw = scene->ui_viz_rw;

    nvgSave(s->vg);

    // path coords are worked out in rgb-box space
    nvgTranslate(s->vg, 240.0f, 0.0);

    // zooom in 2x
    nvgTranslate(s->vg, -1440.0f / 2, -1080.0f / 2);
    nvgScale(s->vg, 2.0, 2.0);

    nvgScale(s->vg, 1440.0f / s->rgb_width, 1080.0f / s->rgb_height);

    nvgBeginPath(s->vg);
    nvgStrokeColor(s->vg, nvgRGBA(255,255,255,128));
    nvgStrokeWidth(s->vg, 1);

    for (int i=box_y; i < box_h; i+=grid_spacing) {
      nvgMoveTo(s->vg, ui_viz_rx, i);
      //nvgLineTo(s->vg, ui_viz_rx, i);
      nvgLineTo(s->vg, ((ui_viz_rw + ui_viz_rx) / 2)+ 10, i);
    }

    for (int i=ui_viz_rx + 12; i <= ui_viz_rw; i+=grid_spacing) {
      nvgMoveTo(s->vg, i, 0);
      nvgLineTo(s->vg, i, 1000);
    }
    nvgStroke(s->vg);
    nvgRestore(s->vg);
  }
}

static void bb_ui_draw_UI(UIState *s) {
  //get 3-state switch position
  int tri_state_fd;
  int tri_state_switch;
  char buffer[10];
  tri_state_switch = 0;
  tri_state_fd = open ("/sys/devices/virtual/switch/tri-state-key/state", O_RDONLY);
  //if we can't open then switch should be considered in the middle, nothing done
  if (tri_state_fd == -1) {
            tri_state_switch = 2;
  } else {
  	read (tri_state_fd, &buffer, 10);
	tri_state_switch = buffer[0] -48;
	close(tri_state_fd);
  }
  if (tri_state_switch == 1) {
	  const UIScene *scene = &s->scene;
	  const int bb_dml_w = 180;
	  const int bb_dml_x =  (scene->ui_viz_rx + (bdr_s*2));
	  const int bb_dml_y = (box_y + (bdr_s*1.5))+220;
	  
	  const int bb_dmr_w = 180;
	  const int bb_dmr_x = scene->ui_viz_rx + scene->ui_viz_rw - bb_dmr_w - (bdr_s*2) ; 
	  const int bb_dmr_y = (box_y + (bdr_s*1.5))+220;

	 bb_ui_draw_measures_left(s,bb_dml_x, bb_dml_y, bb_dml_w );
	 bb_ui_draw_measures_right(s,bb_dmr_x, bb_dmr_y, bb_dmr_w );
   bb_ui_draw_custom_alert(s,"tesla");
	 }
	 if (tri_state_switch ==3) {
	 	ui_draw_vision_grid(s);
	 }
 }
 
 
//BB END: functions added for the display of various items


static void ui_draw_vision_maxspeed(UIState *s) {
  const UIScene *scene = &s->scene;
  int ui_viz_rx = scene->ui_viz_rx;
  int ui_viz_rw = scene->ui_viz_rw;
  float maxspeed = s->scene.v_cruise;

  const int viz_maxspeed_x = (ui_viz_rx + (bdr_s*2));
  const int viz_maxspeed_y = (box_y + (bdr_s*1.5));
  const int viz_maxspeed_w = 180;
  const int viz_maxspeed_h = 202;
  char maxspeed_str[32];
  bool is_cruise_set = (maxspeed != 0 && maxspeed != 255);

  nvgBeginPath(s->vg);
  nvgRoundedRect(s->vg, viz_maxspeed_x, viz_maxspeed_y, viz_maxspeed_w, viz_maxspeed_h, 20);
  nvgStrokeColor(s->vg, nvgRGBA(255,255,255,80));
  nvgStrokeWidth(s->vg, 6);
  nvgStroke(s->vg);

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  nvgFontFace(s->vg, "sans-regular");
  nvgFontSize(s->vg, 26*2.5);
  nvgFillColor(s->vg, nvgRGBA(255, 255, 255, 200));
  nvgText(s->vg, viz_maxspeed_x+viz_maxspeed_w/2, 148, "MAX", NULL);

  nvgFontFace(s->vg, "sans-semibold");
  nvgFontSize(s->vg, 52*2.5);
  nvgFillColor(s->vg, nvgRGBA(255, 255, 255, 255));
  if (is_cruise_set) {
    if (s->is_metric) {
      snprintf(maxspeed_str, sizeof(maxspeed_str), "%d", (int)(maxspeed + 0.5));
    } else {
      snprintf(maxspeed_str, sizeof(maxspeed_str), "%d", (int)(maxspeed * 0.6225 + 0.5));
    }
    nvgText(s->vg, viz_maxspeed_x+viz_maxspeed_w/2, 242, maxspeed_str, NULL);
  } else {
    nvgFontSize(s->vg, 42*2.5);
    nvgText(s->vg, viz_maxspeed_x+viz_maxspeed_w/2, 242, "N/A", NULL);
  }

  //BB START: add new measures panel  const int bb_dml_w = 180;
	bb_ui_draw_UI(s) ;
  //BB END: add new measures panel
}

static void ui_draw_vision_speed(UIState *s) {
  const UIScene *scene = &s->scene;
  int ui_viz_rx = scene->ui_viz_rx;
  int ui_viz_rw = scene->ui_viz_rw;
  float speed = s->scene.v_ego;

  const int viz_speed_w = 280;
  const int viz_speed_x = ui_viz_rx+((ui_viz_rw/2)-(viz_speed_w/2));
  char speed_str[32];

  nvgBeginPath(s->vg);
  nvgRect(s->vg, viz_speed_x, box_y, viz_speed_w, header_h);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

  if (s->is_metric) {
    snprintf(speed_str, sizeof(speed_str), "%d", (int)(speed * 3.6 + 0.5));
  } else {
    snprintf(speed_str, sizeof(speed_str), "%d", (int)(speed * 2.2374144 + 0.5));
  }
  nvgFontFace(s->vg, "sans-bold");
  nvgFontSize(s->vg, 96*2.5);
  nvgFillColor(s->vg, nvgRGBA(255, 255, 255, 255));
  nvgText(s->vg, viz_speed_x+viz_speed_w/2, 240, speed_str, NULL);

  nvgFontFace(s->vg, "sans-regular");
  nvgFontSize(s->vg, 36*2.5);
  nvgFillColor(s->vg, nvgRGBA(255, 255, 255, 200));

  if (s->is_metric) {
    nvgText(s->vg, viz_speed_x+viz_speed_w/2, 320, "kph", NULL);
  } else {
    nvgText(s->vg, viz_speed_x+viz_speed_w/2, 320, "mph", NULL);
  }
}

static void ui_draw_vision_wheel(UIState *s) {
  const UIScene *scene = &s->scene;
  const int ui_viz_rx = scene->ui_viz_rx;
  const int ui_viz_rw = scene->ui_viz_rw;
  const int viz_event_w = 220;
  const int viz_event_x = ((ui_viz_rx + ui_viz_rw) - (viz_event_w + (bdr_s*2)));
  const int viz_event_y = (box_y + (bdr_s*1.5));
  const int viz_event_h = (header_h - (bdr_s*1.5));
  // draw steering wheel
  const int bg_wheel_size = 96;
  const int bg_wheel_x = viz_event_x + (viz_event_w-bg_wheel_size);
  const int bg_wheel_y = viz_event_y + (bg_wheel_size/2);
  const int img_wheel_size = bg_wheel_size*1.5;
  const int img_wheel_x = bg_wheel_x-(img_wheel_size/2);
  const int img_wheel_y = bg_wheel_y-25;
  float img_wheel_alpha = 0.1f;
  bool is_engaged = (s->status == STATUS_ENGAGED);
  bool is_warning = (s->status == STATUS_WARNING);
  bool is_engageable = scene->engageable;
  if (is_engaged || is_warning || is_engageable) {
    nvgBeginPath(s->vg);
    nvgCircle(s->vg, bg_wheel_x, (bg_wheel_y + (bdr_s*1.5)), bg_wheel_size);
    if (is_engaged) {
      nvgFillColor(s->vg, nvgRGBA(23, 134, 68, 255));
    } else if (is_warning) {
      nvgFillColor(s->vg, nvgRGBA(218, 111, 37, 255));
    } else if (is_engageable) {
      nvgFillColor(s->vg, nvgRGBA(23, 51, 73, 255));
    }
    nvgFill(s->vg);
    img_wheel_alpha = 1.0f;
  }
  nvgBeginPath(s->vg);
  NVGpaint imgPaint = nvgImagePattern(s->vg, img_wheel_x, img_wheel_y,
    img_wheel_size, img_wheel_size, 0, s->img_wheel, img_wheel_alpha);
  nvgRect(s->vg, img_wheel_x, img_wheel_y, img_wheel_size, img_wheel_size);
  nvgFillPaint(s->vg, imgPaint);
  nvgFill(s->vg);
}

static void ui_draw_vision_header(UIState *s) {
  const UIScene *scene = &s->scene;
  int ui_viz_rx = scene->ui_viz_rx;
  int ui_viz_rw = scene->ui_viz_rw;

  nvgBeginPath(s->vg);
  NVGpaint gradient = nvgLinearGradient(s->vg, ui_viz_rx,
                        (box_y+(header_h-(header_h/2.5))),
                        ui_viz_rx, box_y+header_h,
                        nvgRGBAf(0,0,0,0.45), nvgRGBAf(0,0,0,0));
  nvgFillPaint(s->vg, gradient);
  nvgRect(s->vg, ui_viz_rx, box_y, ui_viz_rw, header_h);
  nvgFill(s->vg);

  ui_draw_vision_maxspeed(s);
  ui_draw_vision_speed(s);
  ui_draw_vision_wheel(s);
}

static void ui_draw_vision_alert(UIState *s, int va_size, int va_color,
                                  const char* va_text1, const char* va_text2) {
  const UIScene *scene = &s->scene;
  int ui_viz_rx = scene->ui_viz_rx;
  int ui_viz_rw = scene->ui_viz_rw;
  bool hasSidebar = !s->scene.uilayout_sidebarcollapsed;
  bool mapEnabled = s->scene.uilayout_mapenabled;
  bool longAlert1 = strlen(va_text1) > 15;

  const uint8_t *color = alert_colors[va_color];
  const int alr_s = alert_sizes[va_size];
  const int alr_x = ui_viz_rx-(mapEnabled?(hasSidebar?nav_w:(nav_ww)):0)-bdr_s;
  const int alr_w = ui_viz_rw+(mapEnabled?(hasSidebar?nav_w:(nav_ww)):0)+(bdr_s*2);
  const int alr_h = alr_s+(va_size==ALERTSIZE_NONE?0:bdr_s);
  const int alr_y = vwp_h-alr_h;

  nvgBeginPath(s->vg);
  nvgRect(s->vg, alr_x, alr_y, alr_w, alr_h);
  nvgFillColor(s->vg, nvgRGBA(color[0],color[1],color[2],(color[3]*s->alert_blinking_alpha)));
  nvgFill(s->vg);

  nvgBeginPath(s->vg);
  NVGpaint gradient = nvgLinearGradient(s->vg, alr_x, alr_y, alr_x, alr_y+alr_h,
                        nvgRGBAf(0.0,0.0,0.0,0.05), nvgRGBAf(0.0,0.0,0.0,0.35));
  nvgFillPaint(s->vg, gradient);
  nvgRect(s->vg, alr_x, alr_y, alr_w, alr_h);
  nvgFill(s->vg);

  nvgFillColor(s->vg, nvgRGBA(255, 255, 255, 255));
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

  if (va_size == ALERTSIZE_SMALL) {
    nvgFontFace(s->vg, "sans-semibold");
    nvgFontSize(s->vg, 40*2.5);
    nvgText(s->vg, alr_x+alr_w/2, alr_y+alr_h/2+15, va_text1, NULL);
  } else if (va_size== ALERTSIZE_MID) {
    nvgFontFace(s->vg, "sans-bold");
    nvgFontSize(s->vg, 48*2.5);
    nvgText(s->vg, alr_x+alr_w/2, alr_y+alr_h/2-45, va_text1, NULL);
    nvgFontFace(s->vg, "sans-regular");
    nvgFontSize(s->vg, 36*2.5);
    nvgText(s->vg, alr_x+alr_w/2, alr_y+alr_h/2+75, va_text2, NULL);
  } else if (va_size== ALERTSIZE_FULL) {
    nvgFontSize(s->vg, (longAlert1?72:96)*2.5);
    nvgFontFace(s->vg, "sans-bold");
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgTextBox(s->vg, alr_x, alr_y+(longAlert1?360:420), alr_w-60, va_text1, NULL);
    nvgFontSize(s->vg, 48*2.5);
    nvgFontFace(s->vg, "sans-regular");
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
    nvgTextBox(s->vg, alr_x, alr_h-(longAlert1?300:360), alr_w-60, va_text2, NULL);
  }
}

static void ui_draw_calibration_status(UIState *s) {
  const UIScene *scene = &s->scene;
  char calib_str1[64];
  char calib_str2[64];
  snprintf(calib_str1, sizeof(calib_str1), "Calibration in Progress: %d%%", scene->cal_perc);
  snprintf(calib_str2, sizeof(calib_str2), (s->is_metric?"Drive above 72 km/h":"Drive above 45 mph"));

  ui_draw_vision_alert(s, ALERTSIZE_MID, s->status, calib_str1, calib_str2);
}

static void ui_draw_vision(UIState *s) {
  const UIScene *scene = &s->scene;
  int ui_viz_rx = scene->ui_viz_rx;
  int ui_viz_rw = scene->ui_viz_rw;
  int ui_viz_ro = scene->ui_viz_ro;

  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClear(GL_STENCIL_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  // Draw video frames
  glEnable(GL_SCISSOR_TEST);
  glViewport(ui_viz_rx+ui_viz_ro, s->fb_h-(box_y+box_h), viz_w, box_h);
  glScissor(ui_viz_rx, s->fb_h-(box_y+box_h), ui_viz_rw, box_h);
  draw_frame(s);
  glViewport(0, 0, s->fb_w, s->fb_h);
  glDisable(GL_SCISSOR_TEST);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glClear(GL_STENCIL_BUFFER_BIT);

  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  nvgSave(s->vg);

  // Draw augmented elements
  const int inner_height = viz_w*9/16;
  nvgScissor(s->vg, ui_viz_rx, box_y, ui_viz_rw, box_h);
  nvgTranslate(s->vg, ui_viz_rx+ui_viz_ro, box_y + (box_h-inner_height)/2.0);
  nvgScale(s->vg, (float)viz_w / s->fb_w, (float)inner_height / s->fb_h);
  if (!scene->frontview) {
    ui_draw_world(s);
    
  }

  nvgRestore(s->vg);

  // Set Speed, Current Speed, Status/Events
  ui_draw_vision_header(s);

  if (s->scene.alert_size != ALERTSIZE_NONE) {
    // Controls Alerts
    ui_draw_vision_alert(s, s->scene.alert_size, s->status,
                            s->scene.alert_text1, s->scene.alert_text2);
  } else if (scene->cal_status == CALIBRATION_UNCALIBRATED) {
    // Calibration Status
    ui_draw_calibration_status(s);
  }

  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}

static void ui_draw_blank(UIState *s) {
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClear(GL_STENCIL_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
}

static void ui_draw(UIState *s) {
  if (s->vision_connected && s->plus_state == 0) {
    ui_draw_vision(s);
  } else {
    ui_draw_blank(s);
  }

  {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClear(GL_STENCIL_BUFFER_BIT);

    nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);

    nvgEndFrame(s->vg);
    glDisable(GL_BLEND);
  }

  assert(glGetError() == GL_NO_ERROR);
}

static PathData read_path(cereal_ModelData_PathData_ptr pathp) {
  PathData ret = {0};

  struct cereal_ModelData_PathData pathd;
  cereal_read_ModelData_PathData(&pathd, pathp);

  ret.prob = pathd.prob;
  ret.std = pathd.std;

  capn_list32 pointl = pathd.points;
  capn_resolve(&pointl.p);
  for (int i = 0; i < 50; i++) {
    ret.points[i] = capn_to_f32(capn_get32(pointl, i));
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

static void ui_update(UIState *s) {
  int err;

  if (!s->intrinsic_matrix_loaded) {
    s->intrinsic_matrix_loaded = try_load_intrinsics(&s->intrinsic_matrix);
  }

  if (s->vision_connect_firstrun) {
    // cant run this in connector thread because opengl.
    // do this here for now in lieu of a run_on_main_thread event

    for (int i=0; i<UI_BUF_COUNT; i++) {
      glDeleteTextures(1, &s->frame_texs[i]);

      VisionImg img = {
        .fd = s->bufs[i].fd,
        .format = VISIONIMG_FORMAT_RGB24,
        .width = s->rgb_width,
        .height = s->rgb_height,
        .stride = s->rgb_stride,
        .bpp = 3,
        .size = s->rgb_buf_len,
      };
      s->frame_texs[i] = visionimg_to_gl(&img);

      glBindTexture(GL_TEXTURE_2D, s->frame_texs[i]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // BGR
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, GL_GREEN);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, GL_RED);
    }

    for (int i=0; i<UI_BUF_COUNT; i++) {
      glDeleteTextures(1, &s->frame_front_texs[i]);

      VisionImg img = {
        .fd = s->front_bufs[i].fd,
        .format = VISIONIMG_FORMAT_RGB24,
        .width = s->rgb_front_width,
        .height = s->rgb_front_height,
        .stride = s->rgb_front_stride,
        .bpp = 3,
        .size = s->rgb_front_buf_len,
      };
      s->frame_front_texs[i] = visionimg_to_gl(&img);

      glBindTexture(GL_TEXTURE_2D, s->frame_front_texs[i]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // BGR
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, GL_GREEN);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, GL_RED);
    }

    assert(glGetError() == GL_NO_ERROR);

    // Default UI Measurements (Assumes sidebar collapsed)
    s->scene.ui_viz_rx = (box_x-sbr_w+bdr_s*2);
    s->scene.ui_viz_rw = (box_w+sbr_w-(bdr_s*2));
    s->scene.ui_viz_ro = 0;

    s->vision_connect_firstrun = false;

    s->alert_blinking_alpha = 1.0;
    s->alert_blinked = false;
  }

  // poll for events
  while (true) {
    zmq_pollitem_t polls[10] = {{0}};
    polls[0].socket = s->live100_sock_raw;
    polls[0].events = ZMQ_POLLIN;
    polls[1].socket = s->livecalibration_sock_raw;
    polls[1].events = ZMQ_POLLIN;
    polls[2].socket = s->model_sock_raw;
    polls[2].events = ZMQ_POLLIN;
    polls[3].socket = s->live20_sock_raw;
    polls[3].events = ZMQ_POLLIN;
    polls[4].socket = s->livempc_sock_raw;
    polls[4].events = ZMQ_POLLIN;
    polls[5].socket = s->thermal_sock_raw;
    polls[5].events = ZMQ_POLLIN;
    polls[6].socket = s->uilayout_sock_raw;
    polls[6].events = ZMQ_POLLIN;
    polls[7].socket = s->plus_sock_raw;
    polls[7].events = ZMQ_POLLIN;
    polls[8].socket = s->gps_sock_raw;
    polls[8].events = ZMQ_POLLIN;

    int num_polls = 9;
    if (s->vision_connected) {
      assert(s->ipc_fd >= 0);
      polls[9].fd = s->ipc_fd;
      polls[9].events = ZMQ_POLLIN;
      num_polls++;
    }

    int ret = zmq_poll(polls, num_polls, 0);
    if (ret < 0) {
      LOGW("poll failed (%d)", ret);
      break;
    }
    if (ret == 0) {
      break;
    }

    if (polls[0].revents || polls[1].revents || polls[2].revents ||
        polls[3].revents || polls[4].revents || polls[6].revents || polls[7].revents) {
      // awake on any (old) activity
      set_awake(s, true);
    }

    if (s->vision_connected && polls[9].revents) {
      // vision ipc event
      VisionPacket rp;
      err = vipc_recv(s->ipc_fd, &rp);
      if (err <= 0) {
        LOGW("vision disconnected");
        close(s->ipc_fd);
        s->ipc_fd = -1;
        s->vision_connected = false;
        continue;
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
      } else {
        assert(false);
      }
    } else if (polls[7].revents) {
      // plus socket

      zmq_msg_t msg;
      err = zmq_msg_init(&msg);
      assert(err == 0);
      err = zmq_msg_recv(&msg, s->plus_sock_raw, 0);
      assert(err >= 0);

      assert(zmq_msg_size(&msg) == 1);

      s->plus_state = ((char*)zmq_msg_data(&msg))[0];

      zmq_msg_close(&msg);

    } else if (polls[8].revents) 
    {
      // gps socket

      zmq_msg_t msg;
      err = zmq_msg_init(&msg);
      assert(err == 0);
      err = zmq_msg_recv(&msg, s->gps_sock_raw, 0);
      assert(err >= 0);

      struct capn ctx;
      capn_init_mem(&ctx, zmq_msg_data(&msg), zmq_msg_size(&msg), 0);

      cereal_Event_ptr eventp;
      eventp.p = capn_getp(capn_root(&ctx), 0, 1);
      struct cereal_Event eventd;
      cereal_read_Event(&eventd, eventp);

      struct cereal_GpsLocationData datad;
      cereal_read_GpsLocationData(&datad, eventd.gpsLocation);

      s->scene.gpsAccuracy= datad.accuracy;
      if (s->scene.gpsAccuracy>100)
      {
          s->scene.gpsAccuracy=99.99;
      }
      else if (s->scene.gpsAccuracy==0)
      {
          s->scene.gpsAccuracy=99.8;
      }
      capn_free(&ctx);
      zmq_msg_close(&msg);
    } else {
      // zmq messages
      void* which = NULL;
      for (int i=0; i<num_polls - 1; i++) {
        if (polls[i].revents) {
          which = polls[i].socket;
          break;
        }
      }
      if (which == NULL) {
        continue;
      }

      zmq_msg_t msg;
      err = zmq_msg_init(&msg);
      assert(err == 0);
      err = zmq_msg_recv(&msg, which, 0);
      assert(err >= 0);

      struct capn ctx;
      capn_init_mem(&ctx, zmq_msg_data(&msg), zmq_msg_size(&msg), 0);

      cereal_Event_ptr eventp;
      eventp.p = capn_getp(capn_root(&ctx), 0, 1);
      struct cereal_Event eventd;
      cereal_read_Event(&eventd, eventp);

      if (eventd.which == cereal_Event_live100) {
        struct cereal_Live100Data datad;
        cereal_read_Live100Data(&datad, eventd.live100);

        if (datad.vCruise != s->scene.v_cruise) {
          s->scene.v_cruise_update_ts = eventd.logMonoTime;
        }
        s->scene.v_cruise = datad.vCruise;
        s->scene.v_ego = datad.vEgo;
		s->scene.angleSteers = datad.angleSteers;
		s->scene.angleSteersDes = datad.angleSteersDes;
        s->scene.curvature = datad.curvature;
        s->scene.engaged = datad.enabled;
        s->scene.engageable = datad.engageable;
        s->scene.gps_planner_active = datad.gpsPlannerActive;
        // printf("recv %f\n", datad.vEgo);

        s->scene.frontview = datad.rearViewCam;
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
        if (datad.alertSize == cereal_Live100Data_AlertSize_none) {
          s->alert_size = ALERTSIZE_NONE;
        } else if (datad.alertSize == cereal_Live100Data_AlertSize_small) {
          s->alert_size = ALERTSIZE_SMALL;
        } else if (datad.alertSize == cereal_Live100Data_AlertSize_mid) {
          s->alert_size = ALERTSIZE_MID;
        } else if (datad.alertSize == cereal_Live100Data_AlertSize_full) {
          s->alert_size = ALERTSIZE_FULL;
        }

        if (datad.alertStatus == cereal_Live100Data_AlertStatus_userPrompt) {
          update_status(s, STATUS_WARNING);
        } else if (datad.alertStatus == cereal_Live100Data_AlertStatus_critical) {
          update_status(s, STATUS_ALERT);
        } else if (datad.enabled) {
          update_status(s, STATUS_ENGAGED);
        } else {
          update_status(s, STATUS_DISENGAGED);
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

      } else if (eventd.which == cereal_Event_live20) {
        struct cereal_Live20Data datad;
        cereal_read_Live20Data(&datad, eventd.live20);
        struct cereal_Live20Data_LeadData leaddatad;
        cereal_read_Live20Data_LeadData(&leaddatad, datad.leadOne);
        s->scene.lead_status = leaddatad.status;
        s->scene.lead_d_rel = leaddatad.dRel;
        s->scene.lead_y_rel = leaddatad.yRel;
        s->scene.lead_v_rel = leaddatad.vRel;
      } else if (eventd.which == cereal_Event_liveCalibration) {
        s->scene.world_objects_visible = s->intrinsic_matrix_loaded;
        struct cereal_LiveCalibrationData datad;
        cereal_read_LiveCalibrationData(&datad, eventd.liveCalibration);

        s->scene.cal_status = datad.calStatus;
        s->scene.cal_perc = datad.calPerc;

        // should we still even have this?
        capn_list32 warpl = datad.warpMatrix2;
        capn_resolve(&warpl.p);  // is this a bug?
        for (int i = 0; i < 3 * 3; i++) {
          s->scene.warp_matrix.v[i] = capn_to_f32(capn_get32(warpl, i));
        }

        capn_list32 extrinsicl = datad.extrinsicMatrix;
        capn_resolve(&extrinsicl.p);  // is this a bug?
        for (int i = 0; i < 3 * 4; i++) {
          s->scene.extrinsic_matrix.v[i] =
              capn_to_f32(capn_get32(extrinsicl, i));
        }
      } else if (eventd.which == cereal_Event_model) {
        s->scene.model_ts = eventd.logMonoTime;
        s->scene.model = read_model(eventd.model);
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
      } else if (eventd.which == cereal_Event_thermal) {
        struct cereal_ThermalData datad;
        cereal_read_ThermalData(&datad, eventd.thermal);

        if (!datad.started) {
          update_status(s, STATUS_STOPPED);
        } else if (s->status == STATUS_STOPPED) {
          // car is started but controls doesn't have fingerprint yet
          update_status(s, STATUS_DISENGAGED);
        }

        s->scene.started_ts = datad.startedTs;

	   //BBB CPU TEMP
		s->scene.maxCpuTemp=datad.cpu0;
        if (s->scene.maxCpuTemp<datad.cpu1)
        {
            s->scene.maxCpuTemp=datad.cpu1;
        }
        else if (s->scene.maxCpuTemp<datad.cpu2)
        {
            s->scene.maxCpuTemp=datad.cpu2;
        }
        else if (s->scene.maxCpuTemp<datad.cpu3)
        {
            s->scene.maxCpuTemp=datad.cpu3;
        }
      s->scene.maxBatTemp=datad.bat;
	  s->scene.freeSpace=datad.freeSpace;
	   //BBB END CPU TEMP
      } else if (eventd.which == cereal_Event_uiLayoutState) {
          struct cereal_UiLayoutState datad;
          cereal_read_UiLayoutState(&datad, eventd.uiLayoutState);
          s->scene.uilayout_sidebarcollapsed = datad.sidebarCollapsed;
          s->scene.uilayout_mapenabled = datad.mapEnabled;

          bool hasSidebar = !s->scene.uilayout_sidebarcollapsed;
          bool mapEnabled = s->scene.uilayout_mapenabled;
          if (mapEnabled) {
            s->scene.ui_viz_rx = hasSidebar ? (box_x+nav_w) : (box_x+nav_w-(bdr_s*4));
            s->scene.ui_viz_rw = hasSidebar ? (box_w-nav_w) : (box_w-nav_w+(bdr_s*4));
            s->scene.ui_viz_ro = -(sbr_w + 4*bdr_s);
          } else {
            s->scene.ui_viz_rx = hasSidebar ? box_x : (box_x-sbr_w+bdr_s*2);
            s->scene.ui_viz_rw = hasSidebar ? box_w : (box_w+sbr_w-(bdr_s*2));
            s->scene.ui_viz_ro = hasSidebar ? -(sbr_w - 6*bdr_s) : 0;
          }
      }
      capn_free(&ctx);
      zmq_msg_close(&msg);
    }
  }

}

static int vision_subscribe(int fd, VisionPacket *rp, int type) {
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

  UIState *s = args;
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
    pthread_mutex_unlock(&s->lock);
  }
  return NULL;
}


#include <hardware/sensors.h>
#include <utils/Timers.h>

static void* light_sensor_thread(void *args) {
  int err;

  UIState *s = args;
  s->light_sensor = 0.0;

  struct sensors_poll_device_t* device;
  struct sensors_module_t* module;

  hw_get_module(SENSORS_HARDWARE_MODULE_ID, (hw_module_t const**)&module);
  sensors_open(&module->common, &device);

  // need to do this
  struct sensor_t const* list;
  int count = module->get_sensors_list(module, &list);

  int SENSOR_LIGHT = 7;

  struct stat buffer;
  if (stat("/sys/bus/i2c/drivers/cyccg", &buffer) == 0) {
    LOGD("LeEco light sensor detected");
    SENSOR_LIGHT = 5;
  }

  device->activate(device, SENSOR_LIGHT, 0);
  device->activate(device, SENSOR_LIGHT, 1);
  device->setDelay(device, SENSOR_LIGHT, ms2ns(100));

  while (!do_exit) {
    static const size_t numEvents = 1;
    sensors_event_t buffer[numEvents];

    int n = device->poll(device, buffer, numEvents);
    if (n < 0) {
      LOG_100("light_sensor_poll failed: %d", n);
    }
    if (n > 0) {
      if (SENSOR_LIGHT == 5) s->light_sensor = buffer[0].light * 2;
      else s->light_sensor = buffer[0].light;
      //printf("new light sensor value: %f\n", s->light_sensor);
    }
  }

  return NULL;
}


static void* bg_thread(void* args) {
  UIState *s = args;

  EGLDisplay bg_display;
  EGLSurface bg_surface;

  FramebufferState *bg_fb = framebuffer_init("bg", 0x00001000, false,
                              &bg_display, &bg_surface, NULL, NULL);
  assert(bg_fb);

  int bg_status = -1;
  while(!do_exit) {
    pthread_mutex_lock(&s->lock);
    if (bg_status == s->status) {
      // will always be signaled if it changes?
      pthread_cond_wait(&s->bg_cond, &s->lock);
    }
    bg_status = s->status;
    pthread_mutex_unlock(&s->lock);

    assert(bg_status < ARRAYSIZE(bg_colors));
    const uint8_t *color = bg_colors[bg_status];

    glClearColor(color[0]/256.0, color[1]/256.0, color[2]/256.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);

    eglSwapBuffers(bg_display, bg_surface);
    assert(glGetError() == GL_NO_ERROR);
  }

  return NULL;
}


int main() {
  int err;
  setpriority(PRIO_PROCESS, 0, -14);

  zsys_handler_set(NULL);
  signal(SIGINT, (sighandler_t)set_do_exit);

  UIState uistate;
  UIState *s = &uistate;
  ui_init(s);

  pthread_t connect_thread_handle;
  err = pthread_create(&connect_thread_handle, NULL,
                       vision_connect_thread, s);
  assert(err == 0);

  pthread_t light_sensor_thread_handle;
  err = pthread_create(&light_sensor_thread_handle, NULL,
                       light_sensor_thread, s);
  assert(err == 0);

  pthread_t bg_thread_handle;
  err = pthread_create(&bg_thread_handle, NULL,
                       bg_thread, s);
  assert(err == 0);

  TouchState touch = {0};
  touch_init(&touch);

  // light sensor scaling params
  #define LIGHT_SENSOR_M 1.3
  #define LIGHT_SENSOR_B 5.0

  #define NEO_BRIGHTNESS 100

  float smooth_light_sensor = LIGHT_SENSOR_B;

  const int EON = (access("/EON", F_OK) != -1);

  while (!do_exit) {
    bool should_swap = false;
    pthread_mutex_lock(&s->lock);

    if (EON) {
      // light sensor is only exposed on EONs

      float clipped_light_sensor = (s->light_sensor*LIGHT_SENSOR_M) + LIGHT_SENSOR_B;
      if (clipped_light_sensor > 255) clipped_light_sensor = 255;
      smooth_light_sensor = clipped_light_sensor * 0.01 + smooth_light_sensor * 0.99;
      set_brightness(s, (int)smooth_light_sensor);
    } else {
      // compromise for bright and dark envs
      set_brightness(s, NEO_BRIGHTNESS);
    }

    ui_update(s);

    // awake on any touch
    int touch_x = -1, touch_y = -1;
    int touched = touch_poll(&touch, &touch_x, &touch_y, s->awake ? 0 : 100);
    if (touched == 1) {
      // touch event will still happen :(
      set_awake(s, true);
    }

    // manage wakefulness
    if (s->awake_timeout > 0) {
      s->awake_timeout--;
    } else {
      set_awake(s, false);
    }

    if (s->awake) {
      ui_draw(s);
      glFinish();
      should_swap = true;
    }

    pthread_mutex_unlock(&s->lock);

    // the bg thread needs to be scheduled, so the main thread needs time without the lock
    // safe to do this outside the lock?
    if (should_swap) {
      eglSwapBuffers(s->display, s->surface);
  }
  }

  set_awake(s, true);

  // wake up bg thread to exit
  pthread_mutex_lock(&s->lock);
  pthread_cond_signal(&s->bg_cond);
  pthread_mutex_unlock(&s->lock);
  err = pthread_join(bg_thread_handle, NULL);
  assert(err == 0);

  err = pthread_join(connect_thread_handle, NULL);
  assert(err == 0);

  return 0;
}
