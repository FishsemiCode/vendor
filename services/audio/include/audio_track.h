#ifndef __AUDIO_TRACK__
#define __AUDIO_TRACK__

#include <sof/audio/custom.h>

void *audio_track_create(audio_stream_type_t stream);
int audio_track_start(void *track);
int audio_track_stop(void *track);
int audio_track_write(void *track, const void *buffer, uint32_t size);
int audio_track_destroy(void *track);
int audio_track_setparams(void *track,
        int id, enum sof_ipc_frame format, int samplerate, int channels);

#ifndef CONFIG_AUDIO_DAEMON
int audio_service_start(void);
#endif

#endif
