#ifndef COMMON_UTIL_H
#define COMMON_UTIL_H

#include <stdio.h>

#ifndef sighandler_t
typedef void (*sighandler_t)(int sig);
#endif

#ifndef __cplusplus

#define min(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   _a < _b ? _a : _b; })

#define max(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   _a > _b ? _a : _b; })

#endif

#define clamp(a,b,c) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
     __typeof__ (c) _c = (c); \
   _a < _b ? _b : (_a > _c ? _c : _a); })

#define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))

#define ALIGN(x, align) (((x) + (align)-1) & ~((align)-1))

#ifdef __cplusplus
extern "C" {
#endif

// Reads a file into a newly allocated buffer.
//
// Returns NULL on failure, otherwise the NULL-terminated file contents.
// The result must be freed by the caller.
void* read_file(const char* path, size_t* out_len);

void set_thread_name(const char* name);

int set_realtime_priority(int level);

#ifdef __cplusplus
}
#endif

#endif
