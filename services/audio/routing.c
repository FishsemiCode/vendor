#include "routing.h"

/* SCO RX ROUTING */
struct sof_ipc_comp_dai _dais_sco_rx_[] = {
	COMP_DAI(_COMP(101, SOF_COMP_SG_HOST, sof_ipc_comp_dai),
			COMP_CONFIG(3, 3, 2, SOF_IPC_FRAME_S16_LE),
			SOF_IPC_STREAM_CAPTURE, 1),
	COMP_DAI(_COMP(102, SOF_COMP_SG_DAI, sof_ipc_comp_dai),
			COMP_CONFIG(3, 3, 2, SOF_IPC_FRAME_S16_LE),
			SOF_IPC_STREAM_PLAYBACK, 0),
};

struct sof_comp_entry _entries_sco_rx_[] = {
	COMP_ENTRY(_dais_sco_rx_),
};

struct sof_ipc_buffer _buffers_sco_rx_[] = {
	COMP_BUFFER(151, VOICECALL_BUFFSIZE * 2 * 4),
};

struct sof_ipc_pipe_comp_connect _connects_sco_rx_[] = {
	COMP_CONNECT(101, 151),
	COMP_CONNECT(151, 102),
};
/* SCO RX ROUTING END */
/* SCO TX ROUTING */
struct sof_ipc_comp_dai _dais_sco_tx_[] = {
	COMP_DAI(_COMP(201, SOF_COMP_SG_HOST, sof_ipc_comp_dai),
			COMP_CONFIG(3, 3, 2, SOF_IPC_FRAME_S16_LE),
			SOF_IPC_STREAM_CAPTURE, 0),
	COMP_DAI(_COMP(202, SOF_COMP_SG_DAI, sof_ipc_comp_dai),
			COMP_CONFIG(3, 3, 2, SOF_IPC_FRAME_S16_LE),
			SOF_IPC_STREAM_PLAYBACK, 1),
};

struct sof_comp_entry _entries_sco_tx_[] = {
	COMP_ENTRY(_dais_sco_tx_),
};

struct sof_ipc_buffer _buffers_sco_tx_[] = {
	COMP_BUFFER(252, VOICECALL_BUFFSIZE * 2 * 4),
};

struct sof_ipc_pipe_comp_connect _connects_sco_tx_[] = {
	COMP_CONNECT(201, 252),
	COMP_CONNECT(252, 202),
};
/* SCO TX ROUTING END */
/* A2DP ROUTING */
struct sof_ipc_comp_decoder _host_a2dp_[] = {
    COMP_CODEC(_COMP(301, SOF_COMP_DECODER, sof_ipc_comp_decoder)),
};

struct sof_ipc_comp_dai _dais_a2dp_[] = {
	COMP_DAI(
			_COMP(303, SOF_COMP_SG_DAI, sof_ipc_comp_dai),
			COMP_CONFIG(4, 4, 3, SOF_IPC_FRAME_S16_LE),
			SOF_IPC_STREAM_PLAYBACK, 0),
};

struct sof_comp_entry _entries_a2dp_[] = {
	COMP_ENTRY(_dais_a2dp_),
	COMP_ENTRY(_host_a2dp_),
};

struct sof_ipc_buffer _buffers_a2dp_[] = {
	COMP_BUFFER(351, A2DP_BUFFSIZE * 6),
};

struct sof_ipc_pipe_comp_connect _connects_a2dp_[] = {
	COMP_CONNECT(301, 351),
	COMP_CONNECT(351, 303),
};
/* A2DP ROUTING END */

struct pipe_desc _desc_[] = {
    PIPE_DESC(_entries_sco_rx_, _buffers_sco_rx_, _connects_sco_rx_,
            PIPE_NEW(AUDIO_STREAM_BLUETOOTH_SCO_RX, 10000, 10, VOICECALL_BUFFFRAMES), SOF_IPC_STREAM_PLAYBACK),
    PIPE_DESC(_entries_sco_tx_, _buffers_sco_tx_, _connects_sco_tx_,
            PIPE_NEW(AUDIO_STREAM_BLUETOOTH_SCO_TX, 10000, 10, VOICECALL_BUFFFRAMES), SOF_IPC_STREAM_PLAYBACK),
    PIPE_DESC(_entries_a2dp_, _buffers_a2dp_, _connects_a2dp_,
            PIPE_NEW(AUDIO_STREAM_BLUETOOTH_A2DP, 10000, 10, 1280), SOF_IPC_STREAM_PLAYBACK),
};

struct sof_ipc_dai_config _dai_config_[] = {
	DAI_CONFIG(101, SOF_DAI_INTEL_NONE, SOF_DAI_FMT_CBS_CFS | SOF_DAI_FMT_NB_NF | SOF_DAI_FMT_DSP_A),
	DAI_CONFIG(102, SOF_DAI_INTEL_NONE, SOF_DAI_FMT_CBS_CFS | SOF_DAI_FMT_NB_NF | SOF_DAI_FMT_PDM),
	DAI_CONFIG(201, SOF_DAI_INTEL_NONE, SOF_DAI_FMT_CBS_CFS | SOF_DAI_FMT_NB_NF | SOF_DAI_FMT_DSP_A),
	DAI_CONFIG(202, SOF_DAI_INTEL_NONE, SOF_DAI_FMT_CBS_CFS | SOF_DAI_FMT_NB_NF | SOF_DAI_FMT_DSP_A),
	DAI_CONFIG(303, SOF_DAI_INTEL_NONE, SOF_DAI_FMT_CBS_CFS | SOF_DAI_FMT_NB_NF | SOF_DAI_FMT_PDM),
};
