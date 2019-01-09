#include <aservice.h>
#include <message.h>

typedef int32_t (*audio_cb_t) (void *track, void *data, size_t size);

static int audio_message_send_params(void *track,
        uint32_t command, void *opaque, size_t size)
{
    audio_cookie_t cookie = {
        .result = -ENXIO,
        .mutex = PTHREAD_MUTEX_INITIALIZER,
        .cond = PTHREAD_COND_INITIALIZER,
    };
    audio_message_t message;
    void *mq;
    int ret;

    mq = audio_get_message_queue();
    if (!mq)
        return -ENXIO;

    message.track = (uint32_t)track;
    message.command = command;
    message.size = size;
    message.cookie = &cookie;

    if (opaque && size > 0) {
        memcpy(&message.opaque, opaque, size);
    }

    pthread_mutex_lock(&cookie.mutex);

    ret = message_send(mq,
            (message_t *)&message, sizeof(audio_message_t));
    if (ret < 0) {
        pthread_mutex_unlock(&cookie.mutex);
        return ret;
    }

    pthread_cond_wait(&cookie.cond, &cookie.mutex);
    pthread_mutex_unlock(&cookie.mutex);

    return cookie.result;
}

static int audio_message_send(void *track, uint32_t command)
{
    return audio_message_send_params(track, command, NULL, 0);
}

void *audio_track_create(audio_stream_type_t stream)
{
    audio_create_t create;
    int ret;

    create.stream = stream;

    ret = audio_message_send_params(NULL,
            NATIVE_TRACK_CREATE, &create, sizeof(create));
    if (ret <= 0)
        return NULL;

    return (void *)ret;
}

int audio_track_setparams(void *track,
        int id, enum sof_ipc_frame format, int samplerate, int channels)
{
    audio_params_t params;

    params.id = id;
    params.format = format;
    params.samplerate = samplerate;
    params.channels = channels;

    return audio_message_send_params(track,
            NATIVE_TRACK_PARAMS, &params, sizeof(params));
}

int audio_track_start(void *track)
{
    return audio_message_send(track, NATIVE_TRACK_START);
}

int audio_track_stop(void *track)
{
    return audio_message_send(track, NATIVE_TRACK_STOP);
}

int audio_track_write(void *track,
        const void *addr, uint32_t size)
{
    audio_buffer_t buffer;

    buffer.addr = addr;
    buffer.size = size;

    return audio_message_send_params(track,
            NATIVE_TRACK_WRITE, &buffer, sizeof(buffer));
}

int audio_track_destroy(void *track)
{
    return audio_message_send(track, NATIVE_TRACK_DESTROY);
}
