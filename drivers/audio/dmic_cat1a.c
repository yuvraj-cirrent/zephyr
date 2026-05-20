/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Infineon CAT1A AUDIOSS PDM/PCM DMIC driver (V1, CY_IP_MXAUDIOSS).
 *
 * Supports up to 2 channels (stereo PDM bus) on PSoC6-02 devices.
 * Left-channel microphone asserts data on the falling PDM_CLK edge (SELECT=GND).
 * Right-channel microphone asserts data on the rising PDM_CLK edge (SELECT=VCC).
 *
 * DMA is driven via the fixed 1-to-1 trigger
 * TRIG_OUT_1TO1_4_PDM0_RX_TO_PDMA1_TR_IN26 (value 0x40001402).
 * The HF1 clock (fed from the IMO at 8 MHz by default) is used as the PDM
 * clock source.  The driver computes the sinc-decimation rate at configure
 * time from the HF1 frequency and the requested PCM sample rate.
 */

#define DT_DRV_COMPAT infineon_audioss_pdm

#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dmic_cat1a, CONFIG_AUDIO_DMIC_LOG_LEVEL);

#include <cy_pdm_pcm.h>
#include <cy_trigmux.h>
#include <cy_sysclk.h>

/*
 * 1-to-1 trigger line: PDM0 RX -> DW1 channel 26.
 * Defined in the device's trigger mux header; hard-coded here as a fallback
 * to avoid pulling in device-specific headers at driver level.
 */
#ifndef TRIG_OUT_1TO1_4_PDM0_RX_TO_PDMA1_TR_IN26
#define TRIG_OUT_1TO1_4_PDM0_RX_TO_PDMA1_TR_IN26 (0x40001402UL)
#endif

/* Half the maximum V1 FIFO depth (256 words total, shared stereo) */
#define FIFO_TRIGGER_LEVEL (16U)

/* Error interrupt sources (everything except the normal RX-trigger) */
#define PDM_ERR_INTR_MASK (CY_PDM_PCM_INTR_RX_OVERFLOW | CY_PDM_PCM_INTR_RX_UNDERFLOW)

struct ifx_dmic_cat1a_config {
	PDM_Type *reg_addr;
	const struct pinctrl_dev_config *pcfg;
	uint32_t hf_clk_idx; /* HF clock index (e.g. 1 for clk_hf1) */
	void (*irq_config_func)(const struct device *dev);
};

struct ifx_dmic_cat1a_data {
	cy_stc_pdm_pcm_config_t pdm_cfg;
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	struct dma_block_config blk_cfg;
	struct k_mem_slab *mem_slab;
	uint32_t block_size;
	void *active_buf;
	struct k_msgq *rx_queue;
	uint32_t remaining;
	enum dmic_state state;
	uint8_t num_chan;    /* 1 = mono, 2 = stereo */
	uint32_t word_bytes; /* bytes per PCM sample word */
};

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */
static int dmic_cat1a_stop(const struct device *dev);

/* --------------------------------------------------------------------------
 * DMA helpers
 * -------------------------------------------------------------------------- */
static int dmic_cat1a_start_dma(const struct device *dev)
{
	struct ifx_dmic_cat1a_data *data = dev->data;
	const struct ifx_dmic_cat1a_config *config = dev->config;
	uint32_t xfer_size;
	uint32_t offset;
	int ret;

	/*
	 * Each DMA transfer moves FIFO_TRIGGER_LEVEL PCM words.
	 * In stereo mode the FIFO holds interleaved L/R 32-bit words;
	 * each FIFO read yields one word that contains one channel sample.
	 */
	xfer_size = FIFO_TRIGGER_LEVEL * data->word_bytes;
	if (xfer_size > data->remaining) {
		xfer_size = data->remaining;
	}

	offset = data->block_size - data->remaining;

	data->blk_cfg.source_address = (uint32_t)&PDM_PCM_RX_FIFO_RD(config->reg_addr);
	data->blk_cfg.dest_address = (uint32_t)data->active_buf + offset;
	data->blk_cfg.block_size = xfer_size;
	data->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	data->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->blk_cfg.next_block = NULL;

	data->dma_cfg.head_block = &data->blk_cfg;

	ret = dma_config(data->dma_dev, data->dma_channel, &data->dma_cfg);
	if (ret < 0) {
		k_mem_slab_free(data->mem_slab, data->active_buf);
		data->active_buf = NULL;
		return ret;
	}

	ret = dma_start(data->dma_dev, data->dma_channel);
	if (ret < 0) {
		k_mem_slab_free(data->mem_slab, data->active_buf);
		data->active_buf = NULL;
	}

	return ret;
}

static void dma_callback(const struct device *dma_dev, void *arg, uint32_t channel, int status)
{
	struct device *dev = (struct device *)arg;
	struct ifx_dmic_cat1a_data *data = dev->data;
	int ret;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (data->state != DMIC_STATE_ACTIVE) {
		return;
	}

	if (status != 0) {
		LOG_ERR("DMIC DMA error: %d", status);
		dmic_cat1a_stop(dev);
		data->state = DMIC_STATE_ERROR;
		return;
	}

	data->remaining -= FIFO_TRIGGER_LEVEL * data->word_bytes;
	if (data->remaining == 0U) {
		data->remaining = data->block_size;
		if (k_msgq_put(data->rx_queue, &data->active_buf, K_NO_WAIT) != 0) {
			LOG_ERR("DMIC overflow, no space in queue");
			dmic_cat1a_stop(dev);
			data->state = DMIC_STATE_ERROR;
			return;
		}

		ret = k_mem_slab_alloc(data->mem_slab, &data->active_buf, K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("No free memory block for DMIC reception");
			dmic_cat1a_stop(dev);
			data->state = DMIC_STATE_ERROR;
			return;
		}
	}

	ret = dmic_cat1a_start_dma(dev);
	if (ret != 0) {
		LOG_ERR("Failed to restart DMA: %d", ret);
	}
}

/* --------------------------------------------------------------------------
 * Capture start / stop
 * -------------------------------------------------------------------------- */
static int dmic_cat1a_start_capture(const struct device *dev)
{
	struct ifx_dmic_cat1a_data *data = dev->data;
	const struct ifx_dmic_cat1a_config *config = dev->config;
	int ret;

	data->remaining = data->block_size;

	ret = k_mem_slab_alloc(data->mem_slab, &data->active_buf, K_NO_WAIT);
	if (ret != 0) {
		LOG_ERR("No free memory block for DMIC reception");
		data->state = DMIC_STATE_ERROR;
		return -ENOBUFS;
	}

	ret = dmic_cat1a_start_dma(dev);
	if (ret != 0) {
		LOG_ERR("Failed to start DMA: %d", ret);
		return ret;
	}

	Cy_PDM_PCM_ClearInterrupt(config->reg_addr, PDM_ERR_INTR_MASK);
	Cy_PDM_PCM_SetInterruptMask(config->reg_addr, PDM_ERR_INTR_MASK);
	Cy_PDM_PCM_Enable(config->reg_addr);

	data->state = DMIC_STATE_ACTIVE;
	return 0;
}

static int dmic_cat1a_stop(const struct device *dev)
{
	const struct ifx_dmic_cat1a_config *config = dev->config;
	struct ifx_dmic_cat1a_data *data = dev->data;
	void *buf;

	Cy_PDM_PCM_SetInterruptMask(config->reg_addr, 0U);
	Cy_PDM_PCM_Disable(config->reg_addr);

	dma_stop(data->dma_dev, data->dma_channel);

	/* Drain the hardware FIFO */
	while (Cy_PDM_PCM_GetNumInFifo(config->reg_addr) > 0U) {
		(void)Cy_PDM_PCM_ReadFifo(config->reg_addr);
	}

	if (data->active_buf != NULL) {
		k_mem_slab_free(data->mem_slab, data->active_buf);
		data->active_buf = NULL;
	}

	while (k_msgq_get(data->rx_queue, &buf, K_NO_WAIT) == 0) {
		k_mem_slab_free(data->mem_slab, buf);
	}

	return 0;
}

/* --------------------------------------------------------------------------
 * DMIC API
 * -------------------------------------------------------------------------- */
static int ifx_dmic_cat1a_configure(const struct device *dev, struct dmic_cfg *cfg)
{
	const struct ifx_dmic_cat1a_config *config = dev->config;
	struct ifx_dmic_cat1a_data *data = dev->data;
	struct pdm_chan_cfg *channel = &cfg->channel;
	struct pcm_stream_cfg *stream = &cfg->streams[0];
	cy_stc_pdm_pcm_config_t *pdm_cfg = &data->pdm_cfg;
	cy_en_pdm_pcm_status_t cy_ret;
	uint32_t hf1_freq;
	uint32_t mclkq_freq;
	uint32_t sinc_dec;
	cy_en_pdm_pcm_word_len_t word_len;
	uint32_t word_bytes;

	if (data->state == DMIC_STATE_ACTIVE) {
		LOG_ERR("Cannot configure DMIC while active");
		return -EBUSY;
	}

	if (channel->req_num_streams != 1U) {
		return -EINVAL;
	}

	if (channel->req_num_chan < 1U || channel->req_num_chan > 2U) {
		return -EINVAL;
	}

	if (stream->pcm_rate == 0U || stream->mem_slab == NULL || stream->block_size == 0U) {
		return -EINVAL;
	}

	/* Map PCM width to V1 word-length enum */
	switch (stream->pcm_width) {
	case 16:
		word_len = CY_PDM_PCM_WLEN_16_BIT;
		word_bytes = 2U;
		break;
	case 18:
		word_len = CY_PDM_PCM_WLEN_18_BIT;
		word_bytes = 4U;
		break;
	case 20:
		word_len = CY_PDM_PCM_WLEN_20_BIT;
		word_bytes = 4U;
		break;
	case 24:
		word_len = CY_PDM_PCM_WLEN_24_BIT;
		word_bytes = 4U;
		break;
	default:
		LOG_ERR("Unsupported PCM width: %u (V1 PDM supports 16/18/20/24)",
			stream->pcm_width);
		return -EINVAL;
	}

	/* Get HF1 clock frequency */
	hf1_freq = Cy_SysClk_ClkHfGetFrequency(config->hf_clk_idx);

	/*
	 * V1 PDM clock chain:
	 *   HF1 -> clkDiv (bypass=1) -> mclkDiv (1/2 => divide by 2) -> MCLKQ
	 *   MCLKQ = HF1 / 2  (with mclkDiv = CY_PDM_PCM_CLK_DIV_1_2)
	 *
	 * sincDecRate formula (from TRM):
	 *   Fs = F(MCLKQ) / (2 * sincDecRate)
	 *   => sincDecRate = F(MCLKQ) / (2 * Fs)
	 *
	 * sincDecRate must be in range [1, 127].
	 */
	mclkq_freq = hf1_freq / 2U;
	sinc_dec = mclkq_freq / (2U * stream->pcm_rate);

	if (sinc_dec < 1U || sinc_dec > 127U) {
		LOG_ERR("sincDecRate %u out of range [1,127] for Fs=%u Hz, HF1=%u Hz", sinc_dec,
			stream->pcm_rate, hf1_freq);
		return -EINVAL;
	}

	/*
	 * Channel selection:
	 *   1 channel: honour the map (left or right); default to left
	 *   2 channels: stereo
	 */
	cy_en_pdm_pcm_out_t chan_sel;

	if (channel->req_num_chan == 2U) {
		chan_sel = CY_PDM_PCM_OUT_STEREO;
	} else {
		/* inspect the channel map for left/right */
		uint8_t map_idx;
		enum pdm_lr lr;

		dmic_parse_channel_map(channel->req_chan_map_lo, channel->req_chan_map_hi, 0,
				       &map_idx, &lr);
		chan_sel = (lr == PDM_CHAN_RIGHT) ? CY_PDM_PCM_OUT_CHAN_RIGHT
						  : CY_PDM_PCM_OUT_CHAN_LEFT;
	}

	/* Fill V1 PDM config */
	pdm_cfg->clkDiv = CY_PDM_PCM_CLK_DIV_BYPASS;
	pdm_cfg->mclkDiv = CY_PDM_PCM_CLK_DIV_1_2;
	pdm_cfg->ckoDiv = 1U;   /* CKO = MCLKQ / (1+1) = MCLKQ/2 */
	pdm_cfg->ckoDelay = 3U; /* no extra delay */
	pdm_cfg->sincDecRate = (uint8_t)sinc_dec;
	pdm_cfg->chanSelect = chan_sel;
	pdm_cfg->chanSwapEnable = false;
	pdm_cfg->highPassFilterGain = 8U;
	pdm_cfg->highPassDisable = false;
	pdm_cfg->softMuteCycles = CY_PDM_PCM_SOFT_MUTE_CYCLES_64;
	pdm_cfg->softMuteFineGain = 1U;
	pdm_cfg->softMuteEnable = false;
	pdm_cfg->wordLen = word_len;
	pdm_cfg->signExtension = true;
	pdm_cfg->gainLeft = CY_PDM_PCM_BYPASS;
	pdm_cfg->gainRight = CY_PDM_PCM_BYPASS;
	pdm_cfg->rxFifoTriggerLevel = FIFO_TRIGGER_LEVEL - 1U;
	pdm_cfg->dmaTriggerEnable = true;
	pdm_cfg->interruptMask = PDM_ERR_INTR_MASK;

	cy_ret = Cy_PDM_PCM_Init(config->reg_addr, pdm_cfg);
	if (cy_ret != CY_PDM_PCM_SUCCESS) {
		LOG_ERR("Cy_PDM_PCM_Init failed: %d", (int)cy_ret);
		return -EIO;
	}

	/* DMA configuration */
	data->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	data->dma_cfg.source_data_size = word_bytes;
	data->dma_cfg.dest_data_size = word_bytes;
	data->dma_cfg.source_burst_length = 0U;
	data->dma_cfg.dest_burst_length = 0U;
	data->dma_cfg.block_count = 1U;
	data->dma_cfg.complete_callback_en = 1U;
	data->dma_cfg.error_callback_dis = 0U;
	data->dma_cfg.head_block = &data->blk_cfg;
	data->dma_cfg.user_data = (void *)dev;
	data->dma_cfg.dma_callback = dma_callback;

	data->mem_slab = stream->mem_slab;
	data->block_size = stream->block_size;
	data->active_buf = NULL;
	data->word_bytes = word_bytes;
	data->num_chan = (uint8_t)channel->req_num_chan;

	channel->act_num_chan = channel->req_num_chan;
	channel->act_chan_map_lo = channel->req_chan_map_lo;
	channel->act_chan_map_hi = channel->req_chan_map_hi;

	data->state = DMIC_STATE_CONFIGURED;

	LOG_DBG("PDM configured: Fs=%u sinc=%u HF1=%u word=%u-bit", stream->pcm_rate,
		(unsigned int)sinc_dec, hf1_freq, stream->pcm_width);

	return 0;
}

static int ifx_dmic_cat1a_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	struct ifx_dmic_cat1a_data *data = dev->data;
	int ret = 0;

	switch (cmd) {
	case DMIC_TRIGGER_START:
		if (data->state != DMIC_STATE_CONFIGURED) {
			return -EIO;
		}
		ret = dmic_cat1a_start_capture(dev);
		break;

	case DMIC_TRIGGER_RELEASE:
		if (data->state != DMIC_STATE_PAUSED) {
			return -EIO;
		}
		ret = dmic_cat1a_start_capture(dev);
		break;

	case DMIC_TRIGGER_PAUSE:
		data->state = DMIC_STATE_PAUSED;
		dmic_cat1a_stop(dev);
		break;

	case DMIC_TRIGGER_STOP:
		data->state = DMIC_STATE_CONFIGURED;
		dmic_cat1a_stop(dev);
		break;

	case DMIC_TRIGGER_RESET:
		data->state = DMIC_STATE_UNINIT;
		dmic_cat1a_stop(dev);
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static int ifx_dmic_cat1a_read(const struct device *dev, uint8_t stream, void **buffer,
			       size_t *size, int32_t timeout)
{
	struct ifx_dmic_cat1a_data *data = dev->data;
	int ret;

	ARG_UNUSED(stream);

	if ((data->state != DMIC_STATE_ACTIVE) && (data->state != DMIC_STATE_PAUSED)) {
		return -EIO;
	}

	ret = k_msgq_get(data->rx_queue, buffer, SYS_TIMEOUT_MS(timeout));
	if (ret < 0) {
		return ret;
	}

	*size = data->block_size;
	return 0;
}

/* --------------------------------------------------------------------------
 * ISR
 * -------------------------------------------------------------------------- */
static void dmic_cat1a_isr(const struct device *dev)
{
	const struct ifx_dmic_cat1a_config *config = dev->config;
	struct ifx_dmic_cat1a_data *data = dev->data;
	uint32_t status;

	status = Cy_PDM_PCM_GetInterruptStatusMasked(config->reg_addr);

	if (status & CY_PDM_PCM_INTR_RX_OVERFLOW) {
		LOG_ERR("PDM FIFO overflow");
		data->state = DMIC_STATE_ERROR;
	}

	if (status & CY_PDM_PCM_INTR_RX_UNDERFLOW) {
		LOG_ERR("PDM FIFO underflow");
		data->state = DMIC_STATE_ERROR;
	}

	if (data->state == DMIC_STATE_ERROR) {
		dmic_cat1a_stop(dev);
	}

	Cy_PDM_PCM_ClearInterrupt(config->reg_addr, status);
}

/* --------------------------------------------------------------------------
 * Initialisation
 * -------------------------------------------------------------------------- */
static int dmic_cat1a_init(const struct device *dev)
{
	const struct ifx_dmic_cat1a_config *config = dev->config;
	struct ifx_dmic_cat1a_data *data = dev->data;
	int ret;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl_apply_state failed: %d", ret);
		return ret;
	}

	if (!device_is_ready(data->dma_dev)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	/*
	 * Arm the 1-to-1 trigger: PDM0 RX FIFO trigger -> DW1 channel 26.
	 * Cy_TrigMux_Select is used for fixed 1-to-1 lines (PERI v2).
	 */
	if (Cy_TrigMux_Select(TRIG_OUT_1TO1_4_PDM0_RX_TO_PDMA1_TR_IN26, false,
			      TRIGGER_TYPE_LEVEL) != CY_TRIGMUX_SUCCESS) {
		LOG_ERR("Cy_TrigMux_Select failed");
		return -EIO;
	}

	Cy_PDM_PCM_DeInit(config->reg_addr);

	/* Drain any stale FIFO data */
	while (Cy_PDM_PCM_GetNumInFifo(config->reg_addr) > 0U) {
		(void)Cy_PDM_PCM_ReadFifo(config->reg_addr);
	}

	config->irq_config_func(dev);

	data->state = DMIC_STATE_INITIALIZED;

	LOG_DBG("Device %s initialised", dev->name);

	return 0;
}

static DEVICE_API(dmic, dmic_cat1a_api) = {
	.configure = ifx_dmic_cat1a_configure,
	.trigger = ifx_dmic_cat1a_trigger,
	.read = ifx_dmic_cat1a_read,
};

/* --------------------------------------------------------------------------
 * Instance macros
 * -------------------------------------------------------------------------- */
#define DMIC_CAT1A_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static void dmic_cat1a_irq_config_##n(const struct device *dev)                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), dmic_cat1a_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct ifx_dmic_cat1a_config dmic_cat1a_config_##n = {                        \
		.reg_addr = (PDM_Type *)DT_INST_REG_ADDR(n),                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.hf_clk_idx = DT_INST_PROP(n, hf_clock_idx),                                       \
		.irq_config_func = dmic_cat1a_irq_config_##n,                                      \
	};                                                                                         \
                                                                                                   \
	K_MSGQ_DEFINE(dmic_cat1a_rx_queue_##n, sizeof(void *), 8, 4);                              \
                                                                                                   \
	static struct ifx_dmic_cat1a_data dmic_cat1a_data_##n = {                                  \
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, rx)),                        \
		.dma_channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),                          \
		.rx_queue = &dmic_cat1a_rx_queue_##n,                                              \
		.state = DMIC_STATE_UNINIT,                                                        \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, dmic_cat1a_init, NULL, &dmic_cat1a_data_##n,                      \
			      &dmic_cat1a_config_##n, POST_KERNEL,                                 \
			      CONFIG_AUDIO_DMIC_INIT_PRIORITY, &dmic_cat1a_api);

DT_INST_FOREACH_STATUS_OKAY(DMIC_CAT1A_INIT)
