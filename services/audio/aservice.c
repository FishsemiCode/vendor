#include <aservice.h>
#include <message.h>

typedef int32_t (*audio_cb_t) (void *track, void *data, size_t size);

static void *g_audio_mq;

int ipc_stream_send_xrun(struct comp_dev *cdev,
        struct sof_ipc_stream_posn *posn)
{
    return 0;
}

void audio_set_message_queue(void *mq)
{
    g_audio_mq = mq;
}

void *audio_get_message_queue(void)
{
    return g_audio_mq;
}

int32_t native_track_create_handler(
        void *track, void *data, size_t size)
{
    audio_create_t *create = data;
    audio_stream_type_t stream = create->stream;
    struct tplg_comp_dev *tcd;

    tcd = tplg_get_comp(stream);
    if (!tcd)
        return 0;

    if (tcd->refs == 0)
        tcd->refs = 1;
    else {
        syslog(LOG_ERR, "ERROR: can not create stream \"%d\"\n", stream);
        return 0;
    }

    return (int32_t)tcd;
}

static int32_t native_track_params_handler(
        void *track, void *data, size_t size)
{
    struct pipeline *pipe = ((struct tplg_comp_dev *)track)->pipeline;
    struct sof_ipc_stream_params *sparams;
    struct sof_ipc_pcm_params pparams;
    audio_params_t *params = data;
    struct snd_codec codec;
    int ret;

    pparams.comp_id = pipe->ipc_pipe.comp_id;
    sparams = &pparams.params;
    sparams->direction = SOF_IPC_STREAM_PLAYBACK;
    sparams->frame_fmt = params->format;
    sparams->buffer_fmt = SOF_IPC_BUFFER_INTERLEAVED;
    sparams->rate = params->samplerate;
    sparams->channels = params->channels;
    sparams->sample_container_bytes =
        format_to_sample_width(params->format)
        * params->channels / CHAR_BIT;

    ret = pipeline_params(pipe, pipe->sched_comp, &pparams);
    if (ret < 0)
        return ret;

    codec.id = params->id;
    codec.ch_in = params->channels;
    codec.sample_rate = params->samplerate;
    ret = comp_cmd(pipe->sched_comp, COMP_CMD_SET_DATA, &codec);
    if (ret < 0)
        return ret;

    return pipeline_prepare(pipe, pipe->sched_comp);
}

static int32_t native_track_start_handler(
        void *track, void *data, size_t size)
{
    struct pipeline *pipe = ((struct tplg_comp_dev *)track)->pipeline;

    return pipeline_trigger(pipe,
            pipe->sched_comp, COMP_TRIGGER_START);
}

static int32_t native_track_stop_handler(
        void *track, void *data, size_t size)
{
    struct pipeline *pipe = ((struct tplg_comp_dev *)track)->pipeline;

    return pipeline_trigger(pipe,
            pipe->sched_comp, COMP_TRIGGER_STOP);
}

static int32_t native_track_write_handler(
        void *track, void *data, size_t size)
{
    struct pipeline *pipe = ((struct tplg_comp_dev *)track)->pipeline;
    audio_buffer_t *buffer = data;

    return comp_write(pipe->sched_comp,
            buffer->addr, buffer->size);
}

static int32_t native_track_destroy_handler(
        void *track, void *data, size_t size)
{
    struct tplg_comp_dev *tcd = track;

    if (tcd && tcd->refs == 1)
        tcd->refs = 0;

    return 0;
}

static const audio_cb_t g_acb[] = {
    [NATIVE_TRACK_CREATE]   = native_track_create_handler,
    [NATIVE_TRACK_PARAMS]   = native_track_params_handler,
    [NATIVE_TRACK_START]    = native_track_start_handler,
    [NATIVE_TRACK_STOP]     = native_track_stop_handler,
    [NATIVE_TRACK_WRITE]    = native_track_write_handler,
    [NATIVE_TRACK_DESTROY]  = native_track_destroy_handler,
};

static void audio_message_handler(message_t *msg, uint16_t len)
{
    audio_message_t *message = (audio_message_t *)msg;
    audio_cookie_t *cookie = message->cookie;

    if (message->command < ARRAY_SIZE(g_acb))
        cookie->result = g_acb[message->command](
                (void *)message->track, &message->opaque, message->size);

    pthread_mutex_lock(&cookie->mutex);
    pthread_cond_signal(&cookie->cond);
    pthread_mutex_unlock(&cookie->mutex);
}

void *audio_thread(void *arg)
{
    void *mq;

    mq = message_init(audio_message_handler, 0, 0);

    audio_set_message_queue(mq);

    if (tplg_init() < 0) {
        syslog(LOG_ERR, "ETPLG\n");
        return -1;
    }

    message_loop(mq);

    tplg_deinit();

    return NULL;
}

#if !defined(CONFIG_AUDIO_DAEMON)
int audio_service_start(void)
#elif defined(BUILD_MODULE)
int main(int argc, char *argv[])
#else
int aservice_main(int argc, char *argv[])
#endif
{
    void *mq;

    mq = audio_get_message_queue();
    if (mq)
        return true;
#if !defined(CONFIG_AUDIO_DAEMON)
    pthread_t tid;

    return pthread_create(&tid, NULL, audio_thread, NULL);
#else
    audio_thread(NULL);

    return true;
#endif
}
