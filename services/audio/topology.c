#include <nuttx/config.h>
#include <topology.h>

#define COMP_TYPE_COMPONENT	1
#define COMP_TYPE_BUFFER	2
#define COMP_TYPE_PIPELINE	3

static struct list_item comp_list = {
    .next = &comp_list,
    .prev = &comp_list,
};

static int tplg_add_comp(void *comp, int type)
{
    struct tplg_comp_dev *tcd;

    tcd = rzalloc(RZONE_RUNTIME,
            SOF_MEM_CAPS_RAM,
            sizeof(struct tplg_comp_dev));
    if (tcd == NULL)
        return -ENOMEM;

    tcd->cd = comp;
    tcd->type = type;

    list_item_append(&tcd->list, &comp_list);

    return 0;
}

struct tplg_comp_dev *tplg_get_comp(uint32_t id)
{
    struct tplg_comp_dev *tcd;
    struct list_item *list;

    list_for_item(list, &comp_list) {
        tcd = container_of(list, struct tplg_comp_dev, list);
        switch (tcd->type) {
            case COMP_TYPE_COMPONENT:
                if (tcd->cd->comp.id == id)
                    return tcd;
                break;
            case COMP_TYPE_BUFFER:
                if (tcd->cb->ipc_buffer.comp.id == id)
                    return tcd;
                break;
            case COMP_TYPE_PIPELINE:
                if (tcd->pipeline->ipc_pipe.comp_id == id)
                    return tcd;
                break;
            default:
                break;
        }
    }

    return NULL;
}

static void tplg_deinit_comp(void)
{
    struct tplg_comp_dev *tcd;
    struct list_item *list;

    list_for_item(list, &comp_list) {
        tcd = container_of(list, struct tplg_comp_dev, list);
        if (tcd->type != COMP_TYPE_PIPELINE)
            continue;

        pipeline_free(tcd->pipeline);
        list_item_del(&tcd->list);
        rfree(tcd);
    }

    list_for_item(list, &comp_list) {
        tcd = container_of(list, struct tplg_comp_dev, list);
        if (tcd->type == COMP_TYPE_COMPONENT)
            comp_free(tcd->cd);
        else if (tcd->type == COMP_TYPE_BUFFER)
            buffer_free(tcd->cb);

        list_item_del(&tcd->list);
        rfree(tcd);
    }
}

static int tplg_init_comp(void)
{
#include "routing.c"
    struct tplg_comp_dev *tdev, *source, *sink;
    struct sof_ipc_pipe_comp_connect *connect;
    enum sof_ipc_stream_direction dir;
    struct sof_ipc_dai_config *config;
    struct sof_comp_entry *entry;
    struct comp_buffer *buffer;
    struct sof_ipc_comp *comp;
    struct pipe_desc *desc;
    struct pipeline *pipe;
    struct comp_dev *dev;
    int i, j, k, pid;
    int ret, line = 0;

    if (!list_is_empty(&comp_list))
        return -EINVAL;

    for (i = 0; i < ARRAY_SIZE(_desc_); i++) {
        desc = &_desc_[i];

        pid = desc->pipeline.pipeline_id;
        dir = desc->dir;
        pipe = NULL;
        for (j = 0; j < desc->num_entries; j++) {
            entry = &desc->entries[j];

            comp = entry->comps;
            for (k = 0; k < entry->num_comps; k++) {

                comp->pipeline_id = pid;
                dev = comp_new(comp);
                if (!dev) {
                    ret = -ENOMEM;
                    line = __LINE__;
                    goto bail;
                }
                ret = tplg_add_comp(dev, COMP_TYPE_COMPONENT);
                if (ret < 0) {
                    comp_free(dev);
                    line = __LINE__;
                    goto bail;
                }
                if ((comp->type == SOF_COMP_HOST)
                        || (comp->type == (enum sof_comp_type)SOF_COMP_DECODER)
                        || (comp->type == SOF_COMP_SG_HOST
                            && dir != SOF_IPC_STREAM_CAPTURE)) {
                    if (pipe) {
                        ret = -ENOMEM;
                        line = __LINE__;
                        goto bail;
                    }
                    pipe = pipeline_new(&desc->pipeline, dev);
                    if (!pipe) {
                        ret = -ENOMEM;
                        line = __LINE__;
                        goto bail;
                    }
                    ret = tplg_add_comp(pipe, COMP_TYPE_PIPELINE);
                    if (ret < 0) {
                        pipeline_free(pipe);
                        goto bail;
                    }
                }
                comp = (void *)comp + comp->hdr.size;
            }
        }

        if (!pipe) {
            line = __LINE__;
            ret = -ENOMEM;
            goto bail;
        }

        for (j = 0; j < desc->num_buffers; j++) {
            desc->buffer[j].comp.pipeline_id = pid;
            buffer = buffer_new(&desc->buffer[j]);
            if (!buffer) {
                line = __LINE__;
                ret = -ENOMEM;
                goto bail;
            }
            ret = tplg_add_comp(buffer, COMP_TYPE_BUFFER);
            if (ret < 0) {
                line = __LINE__;
                buffer_free(buffer);
                goto bail;
            }
        }

        for (j = 0; j < desc->num_connections; j++) {
            connect = &desc->connect[j];
            source = tplg_get_comp(connect->source_id);
            sink = tplg_get_comp(connect->sink_id);
            if (source->type == COMP_TYPE_BUFFER
                    &&  sink->type == COMP_TYPE_COMPONENT) {
                ret = pipeline_buffer_connect(pipe, source->cb, sink->cd);
                if (ret < 0) {
                    line = __LINE__;
                    goto bail;
                }
            } else if (source->type == COMP_TYPE_COMPONENT &&
                    sink->type == COMP_TYPE_BUFFER) {
                ret = pipeline_comp_connect(pipe, source->cd, sink->cb);
                if (ret < 0) {
                    line = __LINE__;
                    goto bail;
                }
            } else {
                line = __LINE__;
                ret = -EINVAL;
                goto bail;
            }
        }

        ret = pipeline_complete(pipe);
        if (ret < 0) {
            line = __LINE__;
            goto bail;
        }
    }

    for (i = 0; i < ARRAY_SIZE(_dai_config_); i++) {
        config = &_dai_config_[i];
        tdev = tplg_get_comp(config->id);
        if (!tdev) {
            ret = -EINVAL;
            line = __LINE__;
            goto bail;
        }
        if (tdev->type != COMP_TYPE_COMPONENT)
            continue;
        if (tdev->cd->comp.type == SOF_COMP_SG_HOST
                || tdev->cd->comp.type == SOF_COMP_SG_DAI
                || tdev->cd->comp.type == SOF_COMP_HOST
                || tdev->cd->comp.type == SOF_COMP_DAI) {
            ret = comp_dai_config(tdev->cd, config);
            if (ret < 0) {
                line = __LINE__;
                goto bail;
            }
        }
    }

    return 0;

bail:
    syslog(LOG_ERR, "ETPLG:%d\n", line);
    tplg_deinit_comp();
    return ret;
}

void tplg_deinit(void)
{
    tplg_deinit_comp();
}

int tplg_init(void)
{
    sys_comp_init();
    sys_comp_nxdai_init();
    sys_comp_nxhost_init();
    sys_comp_decoder_init();
    sys_comp_mixer_init();

    return tplg_init_comp();
}
