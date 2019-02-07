#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>

#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <dlfcn.h>
#include <time.h>
#include <semaphore.h>
#include <signal.h>
#include <pthread.h>

#include <algorithm>

#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif

#include <libyuv.h>
#include <czmq.h>
#include <capnp/serialize.h>

#include "common/version.h"
#include "common/util.h"
#include "common/timing.h"
#include "common/mat.h"
#include "common/swaglog.h"
#include "common/visionipc.h"
#include "common/visionbuf.h"
#include "common/visionimg.h"
#include "common/buffering.h"

#include "clutil.h"
#include "bufs.h"

#ifdef QCOM
#include "camera_qcom.h"
#else
#include "camera_fake.h"
#endif

#include "model.h"
#include "monitoring.h"

#include "cereal/gen/cpp/log.capnp.h"

#define UI_BUF_COUNT 4

//#define DUMP_RGB

//#define DEBUG_DRIVER_MONITOR

// send net input on port 9000
//#define SEND_NET_INPUT

#define YUV_COUNT 40
#define MAX_CLIENTS 5

#ifdef __APPLE__
typedef void (*sighandler_t) (int);
#endif

extern "C" {
volatile int do_exit = 0;
}

namespace {

struct VisionState;

struct VisionClientState {
  VisionState *s;
  int fd;
  pthread_t thread_handle;
  bool running;
};

struct VisionClientStreamState {
  bool subscribed;
  int bufs_outstanding;
  bool tb;
  TBuffer* tbuffer;
  PoolQueue* queue;
};

struct VisionState {

  int frame_width, frame_height;
  int frame_stride;
  int frame_size;

  int ion_fd;

  // cl state
  cl_device_id device_id;
  cl_context context;

  cl_program prg_debayer_rear;
  cl_program prg_debayer_front;
  cl_kernel krnl_debayer_rear;
  cl_kernel krnl_debayer_front;

  // processing
  TBuffer ui_tb;
  TBuffer ui_front_tb;

  mat3 yuv_transform;
  TBuffer *yuv_tb;

  // TODO: refactor for both cameras?
  Pool yuv_pool;
  VisionBuf yuv_ion[YUV_COUNT];
  cl_mem yuv_cl[YUV_COUNT];
  YUVBuf yuv_bufs[YUV_COUNT];
  FrameMetadata yuv_metas[YUV_COUNT];
  size_t yuv_buf_size;
  int yuv_width, yuv_height;

  // for front camera recording
  Pool yuv_front_pool;
  VisionBuf yuv_front_ion[YUV_COUNT];
  cl_mem yuv_front_cl[YUV_COUNT];
  YUVBuf yuv_front_bufs[YUV_COUNT];
  FrameMetadata yuv_front_metas[YUV_COUNT];
  size_t yuv_front_buf_size;
  int yuv_front_width, yuv_front_height;

  size_t rgb_buf_size;
  int rgb_width, rgb_height, rgb_stride;
  VisionBuf rgb_bufs[UI_BUF_COUNT];
  cl_mem rgb_bufs_cl[UI_BUF_COUNT];

  size_t rgb_front_buf_size;
  int rgb_front_width, rgb_front_height, rgb_front_stride;
  VisionBuf rgb_front_bufs[UI_BUF_COUNT];
  cl_mem rgb_front_bufs_cl[UI_BUF_COUNT];

  ModelState model;
  ModelData model_bufs[UI_BUF_COUNT];

  MonitoringState monitoring;
  zsock_t *monitoring_sock;
  void* monitoring_sock_raw;

  // Protected by transform_lock.
  bool run_model;
  mat3 cur_transform;
  pthread_mutex_t transform_lock;

  cl_mem camera_bufs_cl[FRAME_BUF_COUNT];
  VisionBuf camera_bufs[FRAME_BUF_COUNT];
  VisionBuf focus_bufs[FRAME_BUF_COUNT];
  VisionBuf stats_bufs[FRAME_BUF_COUNT];

  cl_mem front_camera_bufs_cl[FRAME_BUF_COUNT];
  VisionBuf front_camera_bufs[FRAME_BUF_COUNT];

  DualCameraState cameras;

  zsock_t *terminate_pub;
  zsock_t *recorder_sock;
  void* recorder_sock_raw;

  pthread_mutex_t clients_lock;
  VisionClientState clients[MAX_CLIENTS];

};

void hexdump(uint8_t *d, int l) {
  for (int i = 0; i < l; i++) {
    if (i%0x10 == 0 && i != 0) printf("\n");
    printf("%02X ", d[i]);
  }
  printf("\n");
}

int mkpath(char* file_path, mode_t mode) {
  assert(file_path && *file_path);
  char* p;
  for (p=strchr(file_path+1, '/'); p; p=strchr(p+1, '/')) {
    *p='\0';
    if (mkdir(file_path, mode)==-1) {
      if (errno!=EEXIST) { *p='/'; return -1; }
    }
    *p='/';
  }
  return 0;
}

////////// cl stuff

cl_program build_debayer_program(VisionState *s,
                                 int frame_width, int frame_height, int frame_stride,
                                 int rgb_width, int rgb_height, int rgb_stride,
                                 int bayer_flip, int hdr) {
  assert(rgb_width == frame_width/2);
  assert(rgb_height == frame_height/2);

  char args[4096];
  snprintf(args, sizeof(args),
          "-cl-fast-relaxed-math -cl-denorms-are-zero "
          "-DFRAME_WIDTH=%d -DFRAME_HEIGHT=%d -DFRAME_STRIDE=%d "
            "-DRGB_WIDTH=%d -DRGB_HEIGHT=%d -DRGB_STRIDE=%d "
            "-DBAYER_FLIP=%d -DHDR=%d",
          frame_width, frame_height, frame_stride,
          rgb_width, rgb_height, rgb_stride,
          bayer_flip, hdr);
  return CLU_LOAD_FROM_FILE(s->context, s->device_id, "debayer.cl", args);
}

void cl_init(VisionState *s) {
  int err;
  cl_platform_id platform_id = NULL;
  cl_uint num_devices;
  cl_uint num_platforms;

  err = clGetPlatformIDs(1, &platform_id, &num_platforms);
  assert(err == 0);
  err = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_DEFAULT, 1,
                       &s->device_id, &num_devices);
  assert(err == 0);

  cl_print_info(platform_id, s->device_id);
  printf("\n");

  s->context = clCreateContext(NULL, 1, &s->device_id, NULL, NULL, &err);
  assert(err == 0);
}

void cl_free(VisionState *s) {
  int err;

  err = clReleaseContext(s->context);
  assert(err == 0);
}

//////////

#if 0
// from libadreno_utils.so
extern "C" void compute_aligned_width_and_height(int width,
                                      int height,
                                      int bpp,
                                      int tile_mode,
                                      int raster_mode,
                                      int padding_threshold,
                                      int *aligned_w,
                                      int *aligned_h);

// TODO: move to visionbuf
void alloc_rgb888_bufs_cl(cl_device_id device_id, cl_context ctx,
                          int width, int height, int count,
                          int *out_stride, size_t *out_size,
                          VisionBuf *out_bufs, cl_mem *out_cl) {

  int aligned_w = 0, aligned_h = 0;
#ifdef QCOM
  compute_aligned_width_and_height(ALIGN(width, 32), ALIGN(height, 32), 3, 0, 0, 512, &aligned_w, &aligned_h);
#else
  aligned_w = width; aligned_h = height;
#endif

  int stride = aligned_w * 3;
  size_t size = aligned_w * aligned_h * 3;

  for (int i=0; i<count; i++) {
    out_bufs[i] = visionbuf_allocate_cl(size, device_id, ctx,
                                        &out_cl[i]);
  }

  *out_stride = stride;
  *out_size = size;
}
#endif

void init_buffers(VisionState *s) {
  int err;

  // allocate camera buffers

  for (int i=0; i<FRAME_BUF_COUNT; i++) {
    s->camera_bufs[i] = visionbuf_allocate_cl(s->frame_size, s->device_id, s->context,
                                              &s->camera_bufs_cl[i]);
    // TODO: make lengths correct
    s->focus_bufs[i] = visionbuf_allocate(0xb80);
    s->stats_bufs[i] = visionbuf_allocate(0xb80);
  }

  for (int i=0; i<FRAME_BUF_COUNT; i++) {
    s->front_camera_bufs[i] = visionbuf_allocate_cl(s->cameras.front.frame_size,
                                                       s->device_id, s->context,
                                                       &s->front_camera_bufs_cl[i]);
  }

  // processing buffers
  if (s->cameras.rear.ci.bayer) {
    s->rgb_width = s->frame_width/2;
    s->rgb_height = s->frame_height/2;
  } else {
    s->rgb_width = s->frame_width;
    s->rgb_height = s->frame_height;
  }

  for (int i=0; i<UI_BUF_COUNT; i++) {
    VisionImg img = visionimg_alloc_rgb24(s->rgb_width, s->rgb_height, &s->rgb_bufs[i]);
    s->rgb_bufs_cl[i] = visionbuf_to_cl(&s->rgb_bufs[i], s->device_id, s->context);
    if (i == 0){
      s->rgb_stride = img.stride;
      s->rgb_buf_size = img.size;
    }
  }
  tbuffer_init(&s->ui_tb, UI_BUF_COUNT, "rgb");

  assert(s->cameras.front.ci.bayer);
  s->rgb_front_width = s->cameras.front.ci.frame_width/2;
  s->rgb_front_height = s->cameras.front.ci.frame_height/2;

  for (int i=0; i<UI_BUF_COUNT; i++) {
    VisionImg img = visionimg_alloc_rgb24(s->rgb_front_width, s->rgb_front_height, &s->rgb_front_bufs[i]);
    s->rgb_front_bufs_cl[i] = visionbuf_to_cl(&s->rgb_front_bufs[i], s->device_id, s->context);
    if (i == 0){
      s->rgb_front_stride = img.stride;
      s->rgb_front_buf_size = img.size;
    }
  }
  tbuffer_init(&s->ui_front_tb, UI_BUF_COUNT, "frontrgb");

  // yuv back for recording and orbd
  pool_init(&s->yuv_pool, YUV_COUNT);

  s->yuv_tb = pool_get_tbuffer(&s->yuv_pool); //only for visionserver...

  s->yuv_width = s->rgb_width;
  s->yuv_height = s->rgb_height;
  s->yuv_buf_size = s->rgb_width * s->rgb_height * 3 / 2;

  for (int i=0; i<YUV_COUNT; i++) {
    s->yuv_ion[i] = visionbuf_allocate_cl(s->yuv_buf_size, s->device_id, s->context, &s->yuv_cl[i]);
    s->yuv_bufs[i].y = (uint8_t*)s->yuv_ion[i].addr;
    s->yuv_bufs[i].u = s->yuv_bufs[i].y + (s->yuv_width * s->yuv_height);
    s->yuv_bufs[i].v = s->yuv_bufs[i].u + (s->yuv_width/2 * s->yuv_height/2);
  }

  // yuv front for recording
  pool_init(&s->yuv_front_pool, YUV_COUNT);

  s->yuv_front_width = s->rgb_front_width;
  s->yuv_front_height = s->rgb_front_height;
  s->yuv_front_buf_size = s->rgb_front_width * s->rgb_front_height * 3 / 2;

  for (int i=0; i<YUV_COUNT; i++) {
    s->yuv_front_ion[i] = visionbuf_allocate_cl(s->yuv_front_buf_size, s->device_id, s->context, &s->yuv_front_cl[i]);
    s->yuv_front_bufs[i].y = (uint8_t*)s->yuv_front_ion[i].addr;
    s->yuv_front_bufs[i].u = s->yuv_front_bufs[i].y + (s->yuv_front_width * s->yuv_front_height);
    s->yuv_front_bufs[i].v = s->yuv_front_bufs[i].u + (s->yuv_front_width/2 * s->yuv_front_height/2);
  }

  if (s->cameras.rear.ci.bayer) {
    // debayering does a 2x downscale
    s->yuv_transform = transform_scale_buffer(s->cameras.rear.transform, 0.5);
  } else {
    s->yuv_transform = s->cameras.rear.transform;
  }

  // build all the camera debayer programs
  for (int i=0; i<ARRAYSIZE(cameras_supported); i++) {
     int aligned_w, aligned_h;
     visionimg_compute_aligned_width_and_height(cameras_supported[i].frame_width/2, cameras_supported[i].frame_height/2, &aligned_w, &aligned_h);

     build_debayer_program(s, cameras_supported[i].frame_width, cameras_supported[i].frame_height,
                           cameras_supported[i].frame_stride,
                           cameras_supported[i].frame_width/2, cameras_supported[i].frame_height/2,
                           aligned_w*3,
                           cameras_supported[i].bayer_flip, cameras_supported[i].hdr);
  }

  s->prg_debayer_rear = build_debayer_program(s, s->cameras.rear.ci.frame_width, s->cameras.rear.ci.frame_height,
                                                 s->cameras.rear.ci.frame_stride,
                                               s->rgb_width, s->rgb_height, s->rgb_stride,
                                               s->cameras.rear.ci.bayer_flip, s->cameras.rear.ci.hdr);

  s->prg_debayer_front = build_debayer_program(s, s->cameras.front.ci.frame_width, s->cameras.front.ci.frame_height,
                                                  s->cameras.front.ci.frame_stride,
                                               s->rgb_front_width, s->rgb_front_height, s->rgb_front_stride,
                                               s->cameras.front.ci.bayer_flip, s->cameras.front.ci.hdr);

  s->krnl_debayer_rear = clCreateKernel(s->prg_debayer_rear, "debayer10", &err);
  assert(err == 0);
  s->krnl_debayer_front = clCreateKernel(s->prg_debayer_front, "debayer10", &err);
  assert(err == 0);
}

void free_buffers(VisionState *s) {
  // free bufs
  for (int i=0; i<FRAME_BUF_COUNT; i++) {
    visionbuf_free(&s->camera_bufs[i]);
    visionbuf_free(&s->focus_bufs[i]);
    visionbuf_free(&s->stats_bufs[i]);
  }

  for (int i=0; i<FRAME_BUF_COUNT; i++) {
   visionbuf_free(&s->front_camera_bufs[i]);
  }

  for (int i=0; i<UI_BUF_COUNT; i++) {
    visionbuf_free(&s->rgb_bufs[i]);
  }

  for (int i=0; i<UI_BUF_COUNT; i++) {
    visionbuf_free(&s->rgb_front_bufs[i]);
  }

  for (int i=0; i<YUV_COUNT; i++) {
    visionbuf_free(&s->yuv_ion[i]);
  }
}

void* visionserver_client_thread(void* arg) {
  int err;
  VisionClientState *client = (VisionClientState*)arg;
  VisionState *s = client->s;
  int fd = client->fd;

  set_thread_name("clientthread");

  zsock_t *terminate = zsock_new_sub(">inproc://terminate", "");
  assert(terminate);
  void* terminate_raw = zsock_resolve(terminate);

  VisionClientStreamState streams[VISION_STREAM_MAX] = {{0}};

  LOG("client start fd %d\n", fd);

  while (true) {
    zmq_pollitem_t polls[2+VISION_STREAM_MAX] = {{0}};
    polls[0].socket = terminate_raw;
    polls[0].events = ZMQ_POLLIN;
    polls[1].fd = fd;
    polls[1].events = ZMQ_POLLIN;

    int poll_to_stream[2+VISION_STREAM_MAX] = {0};
    int num_polls = 2;
    for (int i=0; i<VISION_STREAM_MAX; i++) {
      if (!streams[i].subscribed) continue;
      polls[num_polls].events = ZMQ_POLLIN;
      if (streams[i].bufs_outstanding >= 2) {
        continue;
      }
      if (streams[i].tb) {
        polls[num_polls].fd = tbuffer_efd(streams[i].tbuffer);
      } else {
        polls[num_polls].fd = poolq_efd(streams[i].queue);
      }
      poll_to_stream[num_polls] = i;
      num_polls++;
    }
    int ret = zmq_poll(polls, num_polls, -1);
    if (ret < 0) {
      LOGE("poll failed (%d)", ret);
      break;
    }
    if (polls[0].revents) {
      break;
    } else if (polls[1].revents) {
      VisionPacket p;
      err = vipc_recv(fd, &p);
      // printf("recv %d\n", p.type);
      if (err <= 0) {
        break;
      } else if (p.type == VIPC_STREAM_SUBSCRIBE) {
        VisionStreamType stream_type = p.d.stream_sub.type;
        VisionPacket rep = {
          .type = VIPC_STREAM_BUFS,
          .d = { .stream_bufs = { .type = stream_type }, },
        };

        VisionClientStreamState *stream = &streams[stream_type];
        stream->tb = p.d.stream_sub.tbuffer;

        VisionStreamBufs *stream_bufs = &rep.d.stream_bufs;
        if (stream_type == VISION_STREAM_RGB_BACK) {
          stream_bufs->width = s->rgb_width;
          stream_bufs->height = s->rgb_height;
          stream_bufs->stride = s->rgb_stride;
          stream_bufs->buf_len = s->rgb_bufs[0].len;
          rep.num_fds = UI_BUF_COUNT;
          for (int i=0; i<rep.num_fds; i++) {
            rep.fds[i] = s->rgb_bufs[i].fd;
          }
          if (stream->tb) {
            stream->tbuffer = &s->ui_tb;
          } else {
            assert(false);
          }
        } else if (stream_type == VISION_STREAM_RGB_FRONT) {
          stream_bufs->width = s->rgb_front_width;
          stream_bufs->height = s->rgb_front_height;
          stream_bufs->stride = s->rgb_front_stride;
          stream_bufs->buf_len = s->rgb_front_bufs[0].len;
          rep.num_fds = UI_BUF_COUNT;
          for (int i=0; i<rep.num_fds; i++) {
            rep.fds[i] = s->rgb_front_bufs[i].fd;
          }
          if (stream->tb) {
            stream->tbuffer = &s->ui_front_tb;
          } else {
            assert(false);
          }
        } else if (stream_type == VISION_STREAM_YUV) {
          stream_bufs->width = s->yuv_width;
          stream_bufs->height = s->yuv_height;
          stream_bufs->stride = s->yuv_width;
          stream_bufs->buf_len = s->yuv_buf_size;
          rep.num_fds = YUV_COUNT;
          for (int i=0; i<rep.num_fds; i++) {
            rep.fds[i] = s->yuv_ion[i].fd;
          }
          if (stream->tb) {
            stream->tbuffer = s->yuv_tb;
          } else {
            stream->queue = pool_get_queue(&s->yuv_pool);
          }
        } else if (stream_type == VISION_STREAM_YUV_FRONT) {
          stream_bufs->width = s->yuv_front_width;
          stream_bufs->height = s->yuv_front_height;
          stream_bufs->stride = s->yuv_front_width;
          stream_bufs->buf_len = s->yuv_front_buf_size;
          rep.num_fds = YUV_COUNT;
          for (int i=0; i<rep.num_fds; i++) {
            rep.fds[i] = s->yuv_front_ion[i].fd;
          }
          if (stream->tb) {
            assert(false);
          } else {
            stream->queue = pool_get_queue(&s->yuv_front_pool);
          }
        } else {
          assert(false);
        }

        if (stream_type == VISION_STREAM_RGB_BACK ||
            stream_type == VISION_STREAM_RGB_FRONT) {
          stream_bufs->buf_info.ui_info = (VisionUIInfo){
            .transformed_width = s->model.in.transformed_width,
            .transformed_height = s->model.in.transformed_height,
          };
        }
        vipc_send(fd, &rep);
        streams[stream_type].subscribed = true;
      } else if (p.type == VIPC_STREAM_RELEASE) {
        // printf("client release f %d  %d\n", p.d.stream_rel.type, p.d.stream_rel.idx);
        int si = p.d.stream_rel.type;
        assert(si < VISION_STREAM_MAX);
        if (streams[si].tb) {
          tbuffer_release(streams[si].tbuffer, p.d.stream_rel.idx);
        } else {
          poolq_release(streams[si].queue, p.d.stream_rel.idx);
        }
        streams[p.d.stream_rel.type].bufs_outstanding--;
      } else {
        assert(false);
      }
    } else {
      int stream_i = VISION_STREAM_MAX;
      for (int i=2; i<num_polls; i++) {
        int si = poll_to_stream[i];
        if (!streams[si].subscribed) continue;
        if (polls[i].revents) {
          stream_i = si;
          break;
        }
      }
      if (stream_i < VISION_STREAM_MAX) {
        streams[stream_i].bufs_outstanding++;
        int idx;
        if (streams[stream_i].tb) {
          idx = tbuffer_acquire(streams[stream_i].tbuffer);
        } else {
          idx = poolq_pop(streams[stream_i].queue);
        }
        if (idx < 0) {
          break;
        }
        VisionPacket rep = {
          .type = VIPC_STREAM_ACQUIRE,
          .d = {.stream_acq = {
            .type = (VisionStreamType)stream_i,
            .idx = idx,
          }},
        };
        if (stream_i == VISION_STREAM_YUV) {
          rep.d.stream_acq.extra.frame_id = s->yuv_metas[idx].frame_id;
          rep.d.stream_acq.extra.timestamp_eof = s->yuv_metas[idx].timestamp_eof;
        } else if (stream_i == VISION_STREAM_YUV_FRONT) {
          rep.d.stream_acq.extra.frame_id = s->yuv_front_metas[idx].frame_id;
          rep.d.stream_acq.extra.timestamp_eof = s->yuv_front_metas[idx].timestamp_eof;
        }
        vipc_send(fd, &rep);
      }
    }
  }

  LOG("client end fd %d\n", fd);

  for (int i=0; i<VISION_STREAM_MAX; i++) {
    if (!streams[i].subscribed) continue;
    if (streams[i].tb) {
      tbuffer_release_all(streams[i].tbuffer);
    } else {
      pool_release_queue(streams[i].queue);
    }
  }

  close(fd);
  zsock_destroy(&terminate);

  pthread_mutex_lock(&s->clients_lock);
  client->running = false;
  pthread_mutex_unlock(&s->clients_lock);

  return NULL;
}

void* visionserver_thread(void* arg) {
  int err;
  VisionState *s = (VisionState*)arg;

  set_thread_name("visionserver");

  zsock_t *terminate = zsock_new_sub(">inproc://terminate", "");
  assert(terminate);
  void* terminate_raw = zsock_resolve(terminate);

  unlink(VIPC_SOCKET_PATH);

  int sock = socket(AF_UNIX, SOCK_SEQPACKET, 0);
  struct sockaddr_un addr = {
    .sun_family = AF_UNIX,
    .sun_path = VIPC_SOCKET_PATH,
  };
  err = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
  assert(err == 0);

  err = listen(sock, 3);
  assert(err == 0);

  // printf("waiting\n");

  while (!do_exit) {
    zmq_pollitem_t polls[2] = {{0}};
    polls[0].socket = terminate_raw;
    polls[0].events = ZMQ_POLLIN;
    polls[1].fd = sock;
    polls[1].events = ZMQ_POLLIN;

    int ret = zmq_poll(polls, ARRAYSIZE(polls), -1);
    if (ret < 0) {
      LOGE("poll failed (%d)", ret);
      break;
    }
    if (polls[0].revents) {
      break;
    } else if (!polls[1].revents) {
      continue;
    }

    int fd = accept(sock, NULL, NULL);
    assert(fd >= 0);

    pthread_mutex_lock(&s->clients_lock);

    int client_idx = 0;
    for (; client_idx < MAX_CLIENTS; client_idx++) {
      if (!s->clients[client_idx].running) break;
    }

    if (client_idx >= MAX_CLIENTS) {
      LOG("ignoring visionserver connection, max clients connected");
      close(fd);

      pthread_mutex_unlock(&s->clients_lock);
      continue;
    }

    VisionClientState *client = &s->clients[client_idx];
    client->s = s;
    client->fd = fd;
    client->running = true;

    err = pthread_create(&client->thread_handle, NULL,
                         visionserver_client_thread, client);
    assert(err == 0);

    pthread_mutex_unlock(&s->clients_lock);
  }

  for (int i=0; i<MAX_CLIENTS; i++) {
    pthread_mutex_lock(&s->clients_lock);
    bool running = s->clients[i].running;
    pthread_mutex_unlock(&s->clients_lock);
    if (running) {
      err = pthread_join(s->clients[i].thread_handle, NULL);
      assert(err == 0);
    }
  }

  close(sock);
  zsock_destroy(&terminate);

  return NULL;
}

void* monitoring_thread(void *arg) {
  int err;
  VisionState *s = (VisionState*)arg;

  set_thread_name("monitoring");

  TBuffer *tb = pool_get_tbuffer(&s->yuv_front_pool);

  cl_command_queue q = clCreateCommandQueue(s->context, s->device_id, 0, &err);
  assert(err == 0);

  double last = 0;
  while (!do_exit) {
    int buf_idx = tbuffer_acquire(tb);
    if (buf_idx < 0) {
      break;
    }

    FrameMetadata frame_data = s->yuv_front_metas[buf_idx];

    // only process every frame
    if ((frame_data.frame_id % 1) == 0) {

      double t1 = millis_since_boot();

      MonitoringResult res = monitoring_eval_frame(&s->monitoring, q,
        s->yuv_front_cl[buf_idx], s->yuv_front_width, s->yuv_front_height);

      // for (int i=0; i<6; i++) {
      //   printf("%f ", res.vs[i]);
      // }
      // printf("\n");

      // send driver monitoring packet
      {
        capnp::MallocMessageBuilder msg;
        cereal::Event::Builder event = msg.initRoot<cereal::Event>();
        event.setLogMonoTime(nanos_since_boot());

        auto framed = event.initDriverMonitoring();
        framed.setFrameId(frame_data.frame_id);

        kj::ArrayPtr<const float> descriptor_vs(&res.vs[0], ARRAYSIZE(res.vs));
        framed.setDescriptor(descriptor_vs);

        framed.setStd(res.std);

        auto words = capnp::messageToFlatArray(msg);
        auto bytes = words.asBytes();
        zmq_send(s->monitoring_sock_raw, bytes.begin(), bytes.size(), ZMQ_DONTWAIT);
      }

      double t2 = millis_since_boot();

      LOGD("monitoring process: %.2fms, from last %.2fms", t2-t1, t1-last);
      last = t1;
    }

    tbuffer_release(tb, buf_idx);
  }

  return NULL;
}

void* frontview_thread(void *arg) {
  int err;
  VisionState *s = (VisionState*)arg;

  set_thread_name("frontview");

  cl_command_queue q = clCreateCommandQueue(s->context, s->device_id, 0, &err);
  assert(err == 0);

  for (int cnt = 0; !do_exit; cnt++) {
    int buf_idx = tbuffer_acquire(&s->cameras.front.camera_tb);
    if (buf_idx < 0) {
      break;
    }

    int ui_idx = tbuffer_select(&s->ui_front_tb);
    FrameMetadata frame_data = s->cameras.front.camera_bufs_metadata[buf_idx];

    double t1 = millis_since_boot();

    err = clSetKernelArg(s->krnl_debayer_front, 0, sizeof(cl_mem), &s->front_camera_bufs_cl[buf_idx]);
    assert(err == 0);
    err = clSetKernelArg(s->krnl_debayer_front, 1, sizeof(cl_mem), &s->rgb_front_bufs_cl[ui_idx]);
    assert(err == 0);
    float digital_gain = 1.0;
    err = clSetKernelArg(s->krnl_debayer_front, 2, sizeof(float), &digital_gain);
    assert(err == 0);

    cl_event debayer_event;
    const size_t debayer_work_size = s->rgb_front_height;
    const size_t debayer_local_work_size = 128;
    err = clEnqueueNDRangeKernel(q, s->krnl_debayer_front, 1, NULL,
                                 &debayer_work_size, &debayer_local_work_size, 0, 0, &debayer_event);
    assert(err == 0);
    clWaitForEvents(1, &debayer_event);
    clReleaseEvent(debayer_event);

    tbuffer_release(&s->cameras.front.camera_tb, buf_idx);

    visionbuf_sync(&s->rgb_front_bufs[ui_idx], VISIONBUF_SYNC_FROM_DEVICE);

    // auto exposure
    const uint8_t *bgr_front_ptr = (const uint8_t*)s->rgb_front_bufs[ui_idx].addr;
#ifndef DEBUG_DRIVER_MONITOR
    if (cnt % 3 == 0)
#endif
    {

      // for driver autoexposure, use bottom right corner
      const int y_start = s->rgb_front_height / 3;
      const int y_end = s->rgb_front_height;
      const int x_start = s->rgb_front_width * 2 / 3;
      const int x_end = s->rgb_front_width;

      uint32_t lum_binning[256] = {0,};
      for (int y = y_start; y < y_end; ++y) {
        for (int x = x_start; x < x_end; x += 2) { // every 2nd col
          const uint8_t *pix = &bgr_front_ptr[y * s->rgb_front_stride + x * 3];
          unsigned int lum = (unsigned int)pix[0] + pix[1] + pix[2];
#ifdef DEBUG_DRIVER_MONITOR
          uint8_t *pix_rw = (uint8_t *)pix;

          // set all the autoexposure pixels to pure green (pixel format is bgr)
          pix_rw[0] = pix_rw[2] = 0;
          pix_rw[1] = 0xff;
#endif
          lum_binning[std::min(lum / 3, 255u)]++;
        }
      }
      const unsigned int lum_total = (y_end - y_start) * (x_end - x_start)/2;
      unsigned int lum_cur = 0;
      int lum_med = 0;
      for (lum_med=0; lum_med<256; lum_med++) {
        lum_cur += lum_binning[lum_med];
        if (lum_cur >= lum_total / 2) {
          break;
        }
      }
      camera_autoexposure(&s->cameras.front, lum_med / 256.0);
    }

    // push YUV buffer
    int yuv_idx = pool_select(&s->yuv_front_pool);
    s->yuv_front_metas[yuv_idx] = frame_data;

    uint8_t *bgr_ptr = (uint8_t*)s->rgb_front_bufs[ui_idx].addr;
    libyuv::RGB24ToI420(bgr_ptr, s->rgb_front_stride,
                        s->yuv_front_bufs[yuv_idx].y, s->yuv_front_width,
                        s->yuv_front_bufs[yuv_idx].u, s->yuv_front_width/2,
                        s->yuv_front_bufs[yuv_idx].v, s->yuv_front_width/2,
                        s->rgb_front_width, s->rgb_front_height);

    s->yuv_front_metas[yuv_idx] = frame_data;
    visionbuf_sync(&s->yuv_front_ion[yuv_idx], VISIONBUF_SYNC_TO_DEVICE);

    // no reference required cause we don't use this in visiond
    //pool_acquire(&s->yuv_front_pool, yuv_idx);
    pool_push(&s->yuv_front_pool, yuv_idx);
    //pool_release(&s->yuv_front_pool, yuv_idx);

    /*FILE *f = fopen("/tmp/test2", "wb");
    printf("%d %d\n", s->rgb_front_height, s->rgb_front_stride);
    fwrite(bgr_front_ptr, 1, s->rgb_front_stride * s->rgb_front_height, f);
    fclose(f);*/

    tbuffer_dispatch(&s->ui_front_tb, ui_idx);

    double t2 = millis_since_boot();

    LOGD("front process: %.2fms", t2-t1);
  }

  return NULL;
}

void* processing_thread(void *arg) {
  int err;
  VisionState *s = (VisionState*)arg;

  set_thread_name("processing");

  err = set_realtime_priority(1);
  LOG("setpriority returns %d", err);

  // init cl stuff
  const cl_queue_properties props[] = {0}; //CL_QUEUE_PRIORITY_KHR, CL_QUEUE_PRIORITY_HIGH_KHR, 0};
  cl_command_queue q = clCreateCommandQueueWithProperties(s->context, s->device_id, props, &err);
  assert(err == 0);

  zsock_t *model_sock = zsock_new_pub("@tcp://*:8009");
  assert(model_sock);
  void *model_sock_raw = zsock_resolve(model_sock);

#ifdef SEND_NET_INPUT
  zsock_t *img_sock = zsock_new_pub("@tcp://*:9000");
  assert(img_sock);
  void *img_sock_raw = zsock_resolve(img_sock);
#else
  void *img_sock_raw = NULL;
#endif

#ifdef DUMP_RGB
  FILE *dump_rgb_file = fopen("/sdcard/dump.rgb", "wb");
#endif

  // init the net
  LOG("processing start!");

  for (int cnt = 0; !do_exit; cnt++) {
    int buf_idx = tbuffer_acquire(&s->cameras.rear.camera_tb);
    // int buf_idx = camera_acquire_buffer(s);
    if (buf_idx < 0) {
      break;
    }

    double t1 = millis_since_boot();

    FrameMetadata frame_data = s->cameras.rear.camera_bufs_metadata[buf_idx];
    uint32_t frame_id = frame_data.frame_id;

    if (frame_id == -1) {
      LOGE("no frame data? wtf");
      tbuffer_release(&s->cameras.rear.camera_tb, buf_idx);
      continue;
    }

    int ui_idx = tbuffer_select(&s->ui_tb);
    int rgb_idx = ui_idx;
    // printf("idx %d\n", rgb_idx);

    /*FILE *f = fopen("/tmp/test_dump", "wb");
    fwrite(s->camera_bufs[buf_idx].addr, 1, s->camera_bufs[buf_idx].len, f);
    fclose(f);*/

    cl_event debayer_event;
    if (s->cameras.rear.ci.bayer) {
      err = clSetKernelArg(s->krnl_debayer_rear, 0, sizeof(cl_mem), &s->camera_bufs_cl[buf_idx]);
      cl_check_error(err);
      err = clSetKernelArg(s->krnl_debayer_rear, 1, sizeof(cl_mem), &s->rgb_bufs_cl[rgb_idx]);
      cl_check_error(err);
      err = clSetKernelArg(s->krnl_debayer_rear, 2, sizeof(float), &s->cameras.rear.digital_gain);
      assert(err == 0);

      const size_t debayer_work_size = s->rgb_height; // doesn't divide evenly, is this okay?
      const size_t debayer_local_work_size = 128;
      err = clEnqueueNDRangeKernel(q, s->krnl_debayer_rear, 1, NULL,
                                   &debayer_work_size, &debayer_local_work_size, 0, 0, &debayer_event);
      assert(err == 0);
    } else {
      assert(s->rgb_buf_size >= s->frame_size);
      assert(s->rgb_stride == s->frame_stride);
      err = clEnqueueCopyBuffer(q, s->camera_bufs_cl[buf_idx], s->rgb_bufs_cl[rgb_idx],
                                0, 0, s->rgb_buf_size, 0, 0, &debayer_event);
      assert(err == 0);
    }

    clWaitForEvents(1, &debayer_event);
    clReleaseEvent(debayer_event);

    tbuffer_release(&s->cameras.rear.camera_tb, buf_idx);

    visionbuf_sync(&s->rgb_bufs[rgb_idx], VISIONBUF_SYNC_FROM_DEVICE);


    double t2 = millis_since_boot();

    uint8_t *bgr_ptr = (uint8_t*)s->rgb_bufs[rgb_idx].addr;

#ifdef DUMP_RGB
    if (cnt % 20 == 0) {
      fwrite(bgr_ptr, s->rgb_buf_size, 1, dump_rgb_file);
    }
#endif

    double yt1 = millis_since_boot();

    int yuv_idx = pool_select(&s->yuv_pool);

    s->yuv_metas[yuv_idx] = frame_data;

    uint8_t* yuv_ptr_y = s->yuv_bufs[yuv_idx].y;
    uint8_t* yuv_ptr_u = s->yuv_bufs[yuv_idx].u;
    uint8_t* yuv_ptr_v = s->yuv_bufs[yuv_idx].v;
    cl_mem yuv_cl = s->yuv_cl[yuv_idx];

    libyuv::RGB24ToI420(bgr_ptr, s->rgb_stride,
                        yuv_ptr_y, s->yuv_width,
                        yuv_ptr_u, s->yuv_width/2,
                        yuv_ptr_v, s->yuv_width/2,
                        s->rgb_width, s->rgb_height);

    double yt2 = millis_since_boot();

    visionbuf_sync(&s->yuv_ion[yuv_idx], VISIONBUF_SYNC_TO_DEVICE);

    // keep another reference around till were done processing
    pool_acquire(&s->yuv_pool, yuv_idx);

    pool_push(&s->yuv_pool, yuv_idx);

    pthread_mutex_lock(&s->transform_lock);
    mat3 transform = s->cur_transform;
    const bool run_model_this_iter = s->run_model;
    pthread_mutex_unlock(&s->transform_lock);

    double mt1 = 0, mt2 = 0;
    if (run_model_this_iter) {

      mat3 model_transform = matmul3(s->yuv_transform, transform);

      mt1 = millis_since_boot();
      s->model_bufs[ui_idx] =
          model_eval_frame(&s->model, q, yuv_cl, s->yuv_width, s->yuv_height,
                           model_transform, img_sock_raw);
      mt2 = millis_since_boot();

      model_publish(model_sock_raw, frame_id, model_transform, s->model_bufs[ui_idx]);
    }

    // send frame event
    {
      capnp::MallocMessageBuilder msg;
      cereal::Event::Builder event = msg.initRoot<cereal::Event>();
      event.setLogMonoTime(nanos_since_boot());

      auto framed = event.initFrame();
      framed.setFrameId(frame_data.frame_id);
      framed.setEncodeId(cnt);
      framed.setTimestampEof(frame_data.timestamp_eof);
      framed.setFrameLength(frame_data.frame_length);
      framed.setIntegLines(frame_data.integ_lines);
      framed.setGlobalGain(frame_data.global_gain);
      framed.setLensPos(frame_data.lens_pos);
      framed.setLensSag(frame_data.lens_sag);
      framed.setLensErr(frame_data.lens_err);
      framed.setLensTruePos(frame_data.lens_true_pos);

#ifndef QCOM
      framed.setImage(kj::arrayPtr((const uint8_t*)s->yuv_ion[yuv_idx].addr, s->yuv_buf_size));
#endif

      kj::ArrayPtr<const float> transform_vs(&s->yuv_transform.v[0], 9);
      framed.setTransform(transform_vs);

      auto words = capnp::messageToFlatArray(msg);
      auto bytes = words.asBytes();
      zmq_send(s->recorder_sock_raw, bytes.begin(), bytes.size(), ZMQ_DONTWAIT);
    }


    tbuffer_dispatch(&s->ui_tb, ui_idx);

    // auto exposure over big box
    const int exposure_x = 290;
    const int exposure_y = 282 + 40;
    const int exposure_height = 314;
    const int exposure_width = 560;
    if (cnt % 3 == 0) {
      // find median box luminance for AE
      uint32_t lum_binning[256] = {0,};
      for (int y=0; y<exposure_height; y++) {
        for (int x=0; x<exposure_width; x++) {
          uint8_t lum = yuv_ptr_y[((exposure_y+y)*s->yuv_width) + exposure_x + x];
          lum_binning[lum]++;
        }
      }
      const unsigned int lum_total = exposure_height * exposure_width;
      unsigned int lum_cur = 0;
      int lum_med = 0;
      for (lum_med=0; lum_med<256; lum_med++) {
        // shouldn't be any values less than 16 - yuv footroom
        lum_cur += lum_binning[lum_med];
        if (lum_cur >= lum_total / 2) {
          break;
        }
      }
      // double avg = (double)acc / (big_box_width * big_box_height) - 16;
      // printf("avg %d\n", lum_med);

      camera_autoexposure(&s->cameras.rear, lum_med / 256.0);
    }

    pool_release(&s->yuv_pool, yuv_idx);

    // if (cnt%40 == 0) {
    //   FILE* of = fopen("/sdcard/tmp.yuv", "wb");
    //   fwrite(transformed_ptr_y, 1, s->transformed_width*s->transformed_height, of);
    //   fwrite(transformed_ptr_u, 1, (s->transformed_width/2)*(s->transformed_height/2), of);
    //   fwrite(transformed_ptr_v, 1, (s->transformed_width/2)*(s->transformed_height/2), of);
    //   fclose(of);
    // }

    double t5 = millis_since_boot();

    LOGD("queued: %.2fms, yuv: %.2f, model: %.2fms | processing: %.3fms",
            (t2-t1), (yt2-yt1), (mt2-mt1), (t5-t1));
  }

#ifdef DUMP_RGB
  fclose(dump_rgb_file);
#endif

  zsock_destroy(&model_sock);

  return NULL;
}

void* live_thread(void *arg) {
  int err;
  VisionState *s = (VisionState*)arg;

  set_thread_name("live");

  zsock_t *terminate = zsock_new_sub(">inproc://terminate", "");
  assert(terminate);

  zsock_t *liveCalibration_sock = zsock_new_sub(">tcp://127.0.0.1:8019", "");
  assert(liveCalibration_sock);

  zpoller_t *poller = zpoller_new(liveCalibration_sock, terminate, NULL);
  assert(poller);

  while (!do_exit) {
    zsock_t *which = (zsock_t*)zpoller_wait(poller, -1);
    if (which == terminate || which == NULL) {
      break;
    }

    zmq_msg_t msg;
    err = zmq_msg_init(&msg);
    assert(err == 0);

    err = zmq_msg_recv(&msg, zsock_resolve(which), 0);
    assert(err >= 0);
    size_t len = zmq_msg_size(&msg);

    // make copy due to alignment issues, will be freed on out of scope
    auto amsg = kj::heapArray<capnp::word>((len / sizeof(capnp::word)) + 1);
    memcpy(amsg.begin(), (const uint8_t*)zmq_msg_data(&msg), len);

    // track camera frames to sync to encoder
    capnp::FlatArrayMessageReader cmsg(amsg);
    cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();

    if (event.isLiveCalibration()) {
      pthread_mutex_lock(&s->transform_lock);
#ifdef BIGMODEL
      auto wm2 = event.getLiveCalibration().getWarpMatrixBig();
#else
      auto wm2 = event.getLiveCalibration().getWarpMatrix2();
#endif
      assert(wm2.size() == 3*3);
      for (int i=0; i<3*3; i++) {
        s->cur_transform.v[i] = wm2[i];
      }
      s->run_model = true;
      pthread_mutex_unlock(&s->transform_lock);
    }

    zmq_msg_close(&msg);
  }

  zpoller_destroy(&poller);
  zsock_destroy(&terminate);

  zsock_destroy(&liveCalibration_sock);

  return NULL;
}

void set_do_exit(int sig) {
  do_exit = 1;
}

void party(VisionState *s, bool nomodel) {
  int err;

  s->terminate_pub = zsock_new_pub("@inproc://terminate");
  assert(s->terminate_pub);

#ifndef __APPLE__
  pthread_t visionserver_thread_handle;
  err = pthread_create(&visionserver_thread_handle, NULL,
                       visionserver_thread, s);
  assert(err == 0);
#endif

  pthread_t proc_thread_handle;
  err = pthread_create(&proc_thread_handle, NULL,
                       processing_thread, s);
  assert(err == 0);


  pthread_t frontview_thread_handle;
  err = pthread_create(&frontview_thread_handle, NULL,
                       frontview_thread, s);
  assert(err == 0);

  pthread_t monitoring_thread_handle;
  err = pthread_create(&monitoring_thread_handle, NULL, monitoring_thread, s);
  assert(err == 0);

  pthread_t live_thread_handle;
  err = pthread_create(&live_thread_handle, NULL,
                       live_thread, s);
  assert(err == 0);

  // priority for cameras
  err = set_realtime_priority(1);
  LOG("setpriority returns %d", err);

  cameras_run(&s->cameras);

  tbuffer_stop(&s->ui_tb);
  tbuffer_stop(&s->ui_front_tb);
  pool_stop(&s->yuv_pool);
  pool_stop(&s->yuv_front_pool);

  zsock_signal(s->terminate_pub, 0);

  LOG("joining frontview_thread");
  err = pthread_join(frontview_thread_handle, NULL);
  assert(err == 0);

#ifndef __APPLE__
  LOG("joining visionserver_thread");
  err = pthread_join(visionserver_thread_handle, NULL);
  assert(err == 0);
#endif

  LOG("joining proc_thread");
  err = pthread_join(proc_thread_handle, NULL);
  assert(err == 0);

  LOG("joining live_thread");
  err = pthread_join(live_thread_handle, NULL);
  assert(err == 0);

  zsock_destroy (&s->terminate_pub);
}

}

// TODO: make a version of visiond that runs on pc using streamed video from EON.
// BOUNTY: free EON+panda+giraffe

int main(int argc, char **argv) {
  int err;

  zsys_handler_set(NULL);
  signal(SIGINT, (sighandler_t)set_do_exit);
  signal(SIGTERM, (sighandler_t)set_do_exit);

  // boringssl via curl via the calibration api can sometimes
  // try to write to a closed socket. just ignore SIGPIPE
  signal(SIGPIPE, SIG_IGN);

  bool test_run = false;
  if (argc > 1 && strcmp(argv[1], "-t") == 0) {
    // immediately tear everything down. useful for caching opencl
    test_run = true;
  }

  bool no_model = false;
  if (argc > 1 && strcmp(argv[1], "--no-model") == 0) {
    no_model = true;
  }

  VisionState state = {0};
  VisionState *s = &state;

  clu_init();
  cl_init(s);

  model_init(&s->model, s->device_id, s->context, true);
  monitoring_init(&s->monitoring, s->device_id, s->context);

  // s->zctx = zctx_shadow_zmq_ctx(zsys_init());

  cameras_init(&s->cameras);

  s->frame_width = s->cameras.rear.ci.frame_width;
  s->frame_height = s->cameras.rear.ci.frame_height;
  s->frame_stride = s->cameras.rear.ci.frame_stride;
  s->frame_size = s->cameras.rear.frame_size;

  // Do not run the model until we receive valid calibration.
  s->run_model = false;
  pthread_mutex_init(&s->transform_lock, NULL);

  init_buffers(s);

  s->recorder_sock = zsock_new_pub("@tcp://*:8002");
  assert(s->recorder_sock);
  s->recorder_sock_raw = zsock_resolve(s->recorder_sock);

  s->monitoring_sock = zsock_new_pub("@tcp://*:8063");
  assert(s->monitoring_sock);
  s->monitoring_sock_raw = zsock_resolve(s->monitoring_sock);

  cameras_open(&s->cameras, &s->camera_bufs[0], &s->focus_bufs[0], &s->stats_bufs[0], &s->front_camera_bufs[0]);

  if (test_run) {
    do_exit = true;
  }
  party(s, no_model);

  zsock_destroy(&s->recorder_sock);
  zsock_destroy(&s->monitoring_sock);
  // zctx_destroy(&s->zctx);

  model_free(&s->model);
  monitoring_free(&s->monitoring);
  free_buffers(s);

  cl_free(s);

  return 0;
}
