#ifndef __AUDIO_TOPOLOGY__
#define __AUDIO_TOPOLOGY__

#include <sof/audio/custom.h>

struct tplg_comp_dev {
    uint16_t type;
    uint16_t state;
    uint16_t refs;
    union {
        void *opaque;
        struct comp_dev *cd;
        struct comp_buffer *cb;
        struct pipeline *pipeline;
    };
    struct list_item list;
};

int tplg_init(void);
void tplg_deinit(void);

struct tplg_comp_dev *tplg_get_comp(uint32_t id);

#endif
