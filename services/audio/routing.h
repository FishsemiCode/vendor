#define VOICECALL_SAMPLERATE  8000
#define VOICECALL_LATENCY      20
#define VOICECALL_BUFFFRAMES     (VOICECALL_SAMPLERATE * VOICECALL_LATENCY / 1000)
#define VOICECALL_BUFFSIZE     (VOICECALL_SAMPLERATE * VOICECALL_LATENCY * sizeof(uint16_t) / 1000)

#define A2DP_SAMPLERATE  44100
#define A2DP_LATENCY     VOICECALL_LATENCY
#define A2DP_CHANNELS    2
#define A2DP_BUFFSIZE    (A2DP_SAMPLERATE * A2DP_LATENCY * sizeof(uint16_t) * A2DP_CHANNELS / 1000)

#define COMP_CONFIG(sink, source, preload, format) \
{ \
	.periods_sink = sink, \
	.periods_source = source, \
	.preload_count = preload, \
	.frame_fmt = format, \
}

#define COMP_BUFFER(cid, csize) \
{ \
	.comp.id = cid, \
	.comp.type = SOF_COMP_BUFFER, \
	.size = csize, \
}

#define _COMP(cid, ctype, csize) \
{\
	.id = cid, \
	.type = ctype, \
	.hdr.size = sizeof(struct csize), \
}

#define COMP_DAI(scomp, sconfig, dir, dev_id) \
{ \
	.comp = scomp, \
	.config = sconfig, \
	.direction = dir, \
	.index = dev_id, \
}

#define COMP_HOST(scomp, sconfig, dir) \
{ \
	.comp = scomp, \
	.config = sconfig, \
	.direction = dir, \
}

#define COMP_CODEC(scomp) \
{ \
	.comp = scomp, \
}

#define COMP_MIXER(scomp, sconfig) \
{ \
	.comp = scomp, \
	.config = sconfig, \
}

#define COMP_CONNECT(source, sink) \
{ \
	.source_id = source, \
	.sink_id = sink, \
}

#define COMP_ENTRY(ccomps) \
    {.comps = (struct sof_ipc_comp *)ccomps, .num_comps = ARRAY_SIZE(ccomps)}

#define PIPE_DESC(sentries, sbuffer, sconnect, spipeline, sdir) \
{ \
	.entries = sentries, \
	.num_entries = ARRAY_SIZE(sentries), \
	.buffer = sbuffer, \
	.num_buffers = ARRAY_SIZE(sbuffer), \
	.connect = sconnect, \
	.num_connections = ARRAY_SIZE(sconnect), \
	.pipeline = spipeline, \
    .dir = sdir, \
}

#define PIPE_NEW(cid, pdeadline, ppriority, frames) \
{ \
	.comp_id = cid, \
	.pipeline_id = cid, \
	.deadline = pdeadline, \
	.priority = ppriority, \
	.frames_per_sched = frames, \
}

#define PCM_PARAMS(cid, dir, ffmt, bfmt, crate, cchannels, sample_bytes) \
{ \
	.comp_id = cid, \
	.params = { \
		.direction = dir, \
		.frame_fmt = ffmt, \
		.buffer_fmt = bfmt, \
		.rate = crate, \
		.channels = cchannels, \
		.sample_container_bytes = sample_bytes, \
	}, \
}

#define DAI_CONFIG(cid, ctype, cformat) \
{ \
	.id = cid, \
	.type = ctype, \
	.format = cformat, \
}

struct sof_comp_entry {
	struct sof_ipc_comp *comps;
	uint32_t num_comps;
};

struct pipe_desc {
	struct sof_comp_entry *entries;
	uint32_t num_entries;
	struct sof_ipc_buffer *buffer;
	uint32_t num_buffers;
	struct sof_ipc_pipe_comp_connect *connect;
	uint32_t num_connections;

	struct sof_ipc_pipe_new pipeline;
    enum sof_ipc_stream_direction dir;
};
