#include "camera_webcam.h"

#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>

#include "common/util.h"
#include "common/timing.h"
#include "common/swaglog.h"
#include "buffering.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include "common/params.h"

extern volatile sig_atomic_t do_exit;
#define FRAME_WIDTH  1164
#define FRAME_HEIGHT 874
#define FRAME_WIDTH_FRONT  1152
#define FRAME_HEIGHT_FRONT 864
namespace {
	
void camera_open(CameraState *s, VisionBuf *camera_bufs, bool rear) {
  printf("open cameras");
  assert(camera_bufs);
  s->camera_bufs = camera_bufs;
}

void camera_close(CameraState *s) {
  tbuffer_stop(&s->camera_tb);
}

void camera_release_buffer(void *cookie, int buf_idx) {
  CameraState *s = static_cast<CameraState *>(cookie);
}


void open_gl_stream_def(CameraState * s, char* camera_id, int width, int height, char ** strm_def, float fx, int flip) {
  printf("OPENGLSTREAM");
  std::string  strm_template="v4l2src device=/dev/v4l/by-id/%s  ! video/x-raw,width=%d,height=%d,framerate=%d/1,format=YUY2 !"
                             " nvvidconv ! video/x-raw(memory:NVMM),format=I420 !"
                             " nvvidconv ! video/x-raw,format=BGRx !"
                             " videoconvert ! video/x-raw,format=BGR !"
                             " videoscale ! video/x-raw,width=%d,height=%d ! %s"
                             " videobox autocrop=true ! video/x-raw,width=%d,height=%d !"
                             " appsink ";
  * strm_def = (char*)calloc(800,1);
  std::string flip_command = "";
  if (flip == 1) {
    flip_command = "videoflip method=rotate-180 ! ";
  }
  sprintf(*strm_def,strm_template.c_str(),camera_id, width, height, s->fps, (int)(s->ci.frame_width*fx), (int)(s->ci.frame_height*fx), flip_command.c_str(), s->ci.frame_width, s->ci.frame_height);
  printf(" GL Stream :[%s]\n",*strm_def);
}

void camera_init(CameraState *s, int camera_id, unsigned int fps) {
  printf("\n\n\n Init Camera %d\n", camera_id);
  assert(camera_id < ARRAYSIZE(cameras_supported));
  s->ci = cameras_supported[camera_id];
  assert(s->ci.frame_width != 0);
   
  s->frame_size = s->ci.frame_height * s->ci.frame_stride;
  s->fps = fps;
  tbuffer_init2(&s->camera_tb, FRAME_BUF_COUNT, "frame", camera_release_buffer, s);
}

static void* rear_thread(void *arg) {
  int err;
  set_thread_name("webcam_rear_thread");
  CameraState* s = (CameraState*)arg;
  char * strm_def;
  printf("open_GL");
  char * cameraId_value;
  int result = read_db_value(NULL, "RoadUsbCameraID", &cameraId_value, NULL);
  char * cameraFx_value;
  result = read_db_value(NULL, "RoadUsbCameraFx", &cameraFx_value, NULL);
  char * cameraFlip_value;
  result = read_db_value(NULL, "RoadUsbCameraFlip", &cameraFlip_value, NULL);
  open_gl_stream_def(s,cameraId_value, 800, 600, &strm_def,atof(cameraFx_value),atoi(cameraFlip_value));
  cv::VideoCapture cap_rear(strm_def);  // road
  free(strm_def);

  cv::Size size;
  size.height = s->ci.frame_height;
  size.width = s->ci.frame_width;

  if (!cap_rear.isOpened()) {
    err = 1;
  }
  uint32_t frame_id = 0;
  TBuffer* tb = &s->camera_tb;
  cv::Mat transformed_mat;
  while (!do_exit) {
    if (cap_rear.read(transformed_mat)) {
     //cv::Size tsize = transformed_mat.size(); 
     //printf("Raw Rear, W=%d, H=%d\n", tsize.width, tsize.height);
      
     int transformed_size = transformed_mat.total() * transformed_mat.elemSize();

      const int buf_idx = tbuffer_select(tb);
      s->camera_bufs_metadata[buf_idx] = {
        .frame_id = frame_id,
      };

      cl_command_queue q = s->camera_bufs[buf_idx].copy_q;
      cl_mem yuv_cl = s->camera_bufs[buf_idx].buf_cl;
      cl_event map_event;
      void *yuv_buf = (void *)clEnqueueMapBuffer(q, yuv_cl, CL_TRUE,
                                                  CL_MAP_WRITE, 0, transformed_size,
                                                  0, NULL, &map_event, &err);
      assert(err == 0);
      clWaitForEvents(1, &map_event);
      clReleaseEvent(map_event);
      memcpy(yuv_buf, transformed_mat.data, transformed_size);

      clEnqueueUnmapMemObject(q, yuv_cl, yuv_buf, 0, NULL, &map_event);
      clWaitForEvents(1, &map_event);
      clReleaseEvent(map_event);
      tbuffer_dispatch(tb, buf_idx);

      frame_id += 1;
  
    }
  }
  transformed_mat.release();
  cap_rear.release();
  return NULL;
}

void front_thread(CameraState *s) {
  int err;
  printf("OPEN FRONT");
  char * strm_def;
  char * cameraId_value;
  int result = read_db_value(NULL, "DriverUsbCameraID", &cameraId_value, NULL);
  char * cameraFx_value;
  result = read_db_value(NULL, "DriverUsbCameraFx", &cameraFx_value, NULL);
  char * cameraFlip_value;
  result = read_db_value(NULL, "DriverUsbCameraFlip", &cameraFlip_value, NULL);
  open_gl_stream_def(s,cameraId_value, 640, 480, &strm_def,atof(cameraFx_value),atoi(cameraFlip_value));
  cv::VideoCapture cap_front(strm_def);  // driver
  free(strm_def);

  cv::Size size;
  size.height = s->ci.frame_height;
  size.width = s->ci.frame_width;

  if (!cap_front.isOpened()) {
    err = 1;
  }

  uint32_t frame_id = 0;
  TBuffer* tb = &s->camera_tb;
  cv::Mat transformed_mat;

  while (!do_exit) {
    if(cap_front.read(transformed_mat)){
      //cv::Size tsize = transformed_mat.size(); 
      //printf("Raw Front, W=%d, H=%d\n", tsize.width, tsize.height);
    

      int transformed_size = transformed_mat.total() * transformed_mat.elemSize();

      const int buf_idx = tbuffer_select(tb);
      s->camera_bufs_metadata[buf_idx] = {
        .frame_id = frame_id,
      };

      cl_command_queue q = s->camera_bufs[buf_idx].copy_q;
      cl_mem yuv_cl = s->camera_bufs[buf_idx].buf_cl;
      cl_event map_event;
      void *yuv_buf = (void *)clEnqueueMapBuffer(q, yuv_cl, CL_TRUE,
                                                  CL_MAP_WRITE, 0, transformed_size,
                                                  0, NULL, &map_event, &err);
      assert(err == 0);
      clWaitForEvents(1, &map_event);
      clReleaseEvent(map_event);
      memcpy(yuv_buf, transformed_mat.data, transformed_size);

      clEnqueueUnmapMemObject(q, yuv_cl, yuv_buf, 0, NULL, &map_event);
      clWaitForEvents(1, &map_event);
      clReleaseEvent(map_event);
      tbuffer_dispatch(tb, buf_idx);

      frame_id += 1;
    }
  }
  transformed_mat.release();
  cap_front.release();
  return;
}

}  // namespace

CameraInfo cameras_supported[CAMERA_ID_MAX] = {
  // road facing
  [CAMERA_ID_LGC920] = {
      .frame_width = FRAME_WIDTH,
      .frame_height = FRAME_HEIGHT,
      .frame_stride = FRAME_WIDTH*3,
      .bayer = false,
      .bayer_flip = false,
  },
  // driver facing
  [CAMERA_ID_LGC615] = {
      .frame_width = FRAME_WIDTH_FRONT,
      .frame_height = FRAME_HEIGHT_FRONT,
      .frame_stride = FRAME_WIDTH_FRONT*3,
      .bayer = false,
      .bayer_flip = false,
  },
};

void cameras_init(DualCameraState *s) {
  memset(s, 0, sizeof(*s));

  camera_init(&s->rear, CAMERA_ID_LGC920, 20);
  s->rear.transform = (mat3){{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  }};
  camera_init(&s->front, CAMERA_ID_LGC615, 10);
  s->front.transform = (mat3){{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
  }};
}

void camera_autoexposure(CameraState *s, float grey_frac) {}

void cameras_open(DualCameraState *s, VisionBuf *camera_bufs_rear,
                  VisionBuf *camera_bufs_focus, VisionBuf *camera_bufs_stats,
                  VisionBuf *camera_bufs_front) {
  assert(camera_bufs_rear);
  assert(camera_bufs_front);
  int err;

  printf("*** open front ***");
  camera_open(&s->front, camera_bufs_front, false);

  printf("*** open rear ***");
  camera_open(&s->rear, camera_bufs_rear, true);
}

void cameras_close(DualCameraState *s) {
  camera_close(&s->rear);
  camera_close(&s->front);
}

void cameras_run(DualCameraState *s) {
  set_thread_name("webcam_thread");
  int err;
  pthread_t rear_thread_handle;
  err = pthread_create(&rear_thread_handle, NULL,
                        rear_thread, &s->rear);
  assert(err == 0);
  front_thread(&s->front);

  err = pthread_join(rear_thread_handle, NULL);
  assert(err == 0);
  cameras_close(s);
}
