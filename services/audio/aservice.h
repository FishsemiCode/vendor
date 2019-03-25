#ifndef __AUDIO_SERVICE__
#define __AUDIO_SERVICE__

#include <nuttx/config.h>
#include <syslog.h>

#include "topology.h"
#include "audio_track.h"

#define NATIVE_TRACK_CREATE    1
#define NATIVE_TRACK_PARAMS    2
#define NATIVE_TRACK_START     3
#define NATIVE_TRACK_STOP      4
#define NATIVE_TRACK_WRITE     5
#define NATIVE_TRACK_DESTROY   6

typedef struct {
    int32_t             result;
    pthread_mutex_t     mutex;
    pthread_cond_t      cond;
} audio_cookie_t;

typedef struct {
    int                 id;
    enum sof_ipc_frame  format;
    int                 samplerate;
    int                 channels;
} audio_params_t;

typedef struct {
    const void         *addr;
    size_t              size;
} audio_buffer_t;

typedef struct {
    audio_stream_type_t stream;
} audio_create_t;

typedef struct {
    uint32_t            track;
    uint32_t            command;
    uint32_t            size;
    audio_cookie_t     *cookie;
    union {
        void           *opaque;
        audio_params_t  params;
        audio_buffer_t  buffer;
        audio_create_t  create;
    };
} audio_message_t;

void audio_set_message_queue(void *mq);
void *audio_get_message_queue(void);

#endif
