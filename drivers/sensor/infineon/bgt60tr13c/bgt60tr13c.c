/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_bgt60tr13c

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/sys/byteorder.h>

#include "bgt60tr13c.h"

LOG_MODULE_REGISTER(bgt60tr13c, CONFIG_SENSOR_LOG_LEVEL);

static int bgt60tr13c_reg_read(const struct device *dev, uint8_t reg, uint32_t *val)
{
	const struct bgt60tr13c_config *cfg = dev->config;
	uint8_t tx_buf[4];
	uint8_t rx_buf[4];

	/*
	 * 32-bit SPI frame: byte[0]=(addr<<1)|rw, bytes[1:3]=24-bit data.
	 * rw=0 for read.  rx[0]=GSR0 status, rx[1:3]=data[23:0].
	 * Return full 24-bit value so callers can preserve upper-byte bits.
	 */
	tx_buf[0] = (reg << 1) | BGT60TR13C_SPI_READ_BIT;
	tx_buf[1] = 0U;
	tx_buf[2] = 0U;
	tx_buf[3] = 0U;

	const struct spi_buf tx = {.buf = tx_buf, .len = 4};
	const struct spi_buf rx = {.buf = rx_buf, .len = 4};
	const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

	int ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);

	if (ret < 0) {
		return ret;
	}

	*val = ((uint32_t)rx_buf[1] << 16) | ((uint32_t)rx_buf[2] << 8) | rx_buf[3];
	return 0;
}

static int __maybe_unused bgt60tr13c_reg_write(const struct device *dev, uint8_t reg, uint32_t val)
{
	const struct bgt60tr13c_config *cfg = dev->config;
	uint8_t tx_buf[4];

	/*
	 * 32-bit SPI frame: byte[0]=(addr<<1)|rw, bytes[1:3]=24-bit data.
	 * rw=1 for write.  Full 24-bit value preserved (upper byte included).
	 */
	tx_buf[0] = (reg << 1) | BGT60TR13C_SPI_WRITE_BIT;
	tx_buf[1] = (uint8_t)((val >> 16) & 0xFFU);
	tx_buf[2] = (uint8_t)((val >> 8) & 0xFFU);
	tx_buf[3] = (uint8_t)(val & 0xFFU);

	const struct spi_buf tx = {.buf = tx_buf, .len = 4};
	const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};

	return spi_write_dt(&cfg->spi, &tx_set);
}

static int bgt60tr13c_fifo_read(const struct device *dev, uint16_t *buf, uint16_t count)
{
	const struct bgt60tr13c_config *cfg = dev->config;

	/*
	 * FIFO burst read (single CS assertion):
	 *   4-byte header {0xFF, 0xC0, 0x00, 0x00}:
	 *     0xFF = all-ones burst mode indicator
	 *     0xC0 = (REG_FIFO_DATA << 1) = SADR with rw=0 (read)
	 *   CS held low; then count*2 bytes of ADC sample data follow.
	 *   rx_hdr[0] = GSR0 status — checked for errors before using data.
	 *   ADC samples are big-endian on the wire; swap to CPU order.
	 */
	uint8_t tx_hdr[4] = {
		BGT60TR13C_BURST_HDR_B0,
		BGT60TR13C_BURST_HDR_B1,
		0U,
		0U,
	};
	uint8_t rx_hdr[4];

	const struct spi_buf tx_bufs[] = {
		{.buf = tx_hdr, .len = 4},
		{.buf = NULL, .len = count * 2U},
	};
	const struct spi_buf rx_bufs[] = {
		{.buf = rx_hdr, .len = 4},
		{.buf = buf, .len = count * 2U},
	};
	const struct spi_buf_set tx_set = {.buffers = tx_bufs, .count = 2};
	const struct spi_buf_set rx_set = {.buffers = rx_bufs, .count = 2};

	int ret = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);

	if (ret < 0) {
		return ret;
	}

	/* GSR0 error bits: FOU_ERR=bit0, SPI_BURST_ERR=bit2, CLK_NUM_ERR=bit3 */
	if (rx_hdr[0] & (0x01U | 0x04U | 0x08U)) {
		LOG_WRN("FIFO burst GSR0 error: 0x%02x", rx_hdr[0]);
		return -EIO;
	}

	/* Swap big-endian wire bytes to CPU (little-endian) order. */
	for (uint16_t i = 0U; i < count; i++) {
		buf[i] = sys_be16_to_cpu(buf[i]);
	}

	return 0;
}

static int bgt60tr13c_hw_reset(const struct device *dev)
{
	const struct bgt60tr13c_config *cfg = dev->config;
	int ret;

	/* Assert reset (active-low) */
	ret = gpio_pin_set_dt(&cfg->reset_gpio, 1);
	if (ret < 0) {
		return ret;
	}

	k_usleep(100);

	/* De-assert reset */
	ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);
	if (ret < 0) {
		return ret;
	}

	/* Wait for sensor to become ready after reset */
	k_msleep(2);

	return 0;
}

#ifdef CONFIG_BGT60TR13C_TRIGGER
static void bgt60tr13c_irq_callback(const struct device *port, struct gpio_callback *cb,
				    gpio_port_pins_t pins)
{
	struct bgt60tr13c_data *data = CONTAINER_OF(cb, struct bgt60tr13c_data, irq_cb);

#ifdef CONFIG_BGT60TR13C_TRIGGER_OWN_THREAD
	k_sem_give(&data->irq_sem);
#elif defined(CONFIG_BGT60TR13C_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif
}

static void bgt60tr13c_handle_trigger(const struct device *dev)
{
	struct bgt60tr13c_data *data = dev->data;

	if (data->data_ready_handler != NULL) {
		data->data_ready_handler(dev, data->data_ready_trigger);
	}
}

#ifdef CONFIG_BGT60TR13C_TRIGGER_OWN_THREAD
static void bgt60tr13c_thread_main(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct bgt60tr13c_data *data = p1;

	while (true) {
		k_sem_take(&data->irq_sem, K_FOREVER);
		bgt60tr13c_handle_trigger(data->dev);
	}
}
#elif defined(CONFIG_BGT60TR13C_TRIGGER_GLOBAL_THREAD)
static void bgt60tr13c_work_handler(struct k_work *work)
{
	struct bgt60tr13c_data *data = CONTAINER_OF(work, struct bgt60tr13c_data, work);

	bgt60tr13c_handle_trigger(data->dev);
}
#endif

static int bgt60tr13c_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
				  sensor_trigger_handler_t handler)
{
	struct bgt60tr13c_data *data = dev->data;
	const struct bgt60tr13c_config *cfg = dev->config;

	if (trig->type != SENSOR_TRIG_DATA_READY) {
		return -ENOTSUP;
	}

	data->data_ready_handler = handler;
	data->data_ready_trigger = trig;

	/* Enable or disable interrupt */
	if (handler != NULL) {
		return gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	} else {
		return gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
	}
}

static int bgt60tr13c_trigger_init(const struct device *dev)
{
	struct bgt60tr13c_data *data = dev->data;
	const struct bgt60tr13c_config *cfg = dev->config;
	int ret;

	if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
		LOG_ERR("IRQ GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure IRQ GPIO: %d", ret);
		return ret;
	}

	gpio_init_callback(&data->irq_cb, bgt60tr13c_irq_callback, BIT(cfg->irq_gpio.pin));

	ret = gpio_add_callback(cfg->irq_gpio.port, &data->irq_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add IRQ callback: %d", ret);
		return ret;
	}

	data->dev = dev;

#ifdef CONFIG_BGT60TR13C_TRIGGER_OWN_THREAD
	k_sem_init(&data->irq_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&data->thread, data->thread_stack, CONFIG_BGT60TR13C_THREAD_STACK_SIZE,
			bgt60tr13c_thread_main, data, NULL, NULL,
			K_PRIO_COOP(CONFIG_BGT60TR13C_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_BGT60TR13C_TRIGGER_GLOBAL_THREAD)
	k_work_init(&data->work, bgt60tr13c_work_handler);
#endif

	return 0;
}
#endif /* CONFIG_BGT60TR13C_TRIGGER */

static int bgt60tr13c_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct bgt60tr13c_data *data = dev->data;
	uint32_t fstat;
	int ret;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	/* Read FIFO status to get sample count */
	ret = bgt60tr13c_reg_read(dev, BGT60TR13C_REG_FSTAT, &fstat);
	if (ret < 0) {
		LOG_ERR("Failed to read FIFO status: %d", ret);
		return ret;
	}

	LOG_DBG("FSTAT raw=0x%06x (reg 0x%02x)", fstat, BGT60TR13C_REG_FSTAT);

	data->fifo_count = fstat & BGT60TR13C_FSTAT_FILL_MSK;
	if (data->fifo_count == 0U) {
		return 0;
	}

	if (data->fifo_count > BGT60TR13C_FIFO_FETCH_MAX) {
		data->fifo_count = BGT60TR13C_FIFO_FETCH_MAX;
	}

	/* Burst read FIFO data */
	ret = bgt60tr13c_fifo_read(dev, data->fifo_buf, data->fifo_count);
	if (ret < 0) {
		LOG_ERR("Failed to read FIFO data: %d", ret);
		return ret;
	}

	return 0;
}

static int bgt60tr13c_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct bgt60tr13c_data *data = dev->data;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	/* Return FIFO count in val1, first sample in val2 */
	val->val1 = (int32_t)data->fifo_count;
	val->val2 = (data->fifo_count > 0U) ? (int32_t)data->fifo_buf[0] : 0;

	return 0;
}

static int bgt60tr13c_init(const struct device *dev)
{
	const struct bgt60tr13c_config *cfg = dev->config;
	uint32_t chip_id;
	uint32_t main_val;
	int ret;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
		LOG_ERR("Reset GPIO device not ready");
		return -ENODEV;
	}

	/* Configure reset GPIO */
	ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure reset GPIO: %d", ret);
		return ret;
	}

	/* Perform hardware reset */
	ret = bgt60tr13c_hw_reset(dev);
	if (ret < 0) {
		LOG_ERR("Hardware reset failed: %d", ret);
		return ret;
	}

	/* Read and verify chip ID */
	ret = bgt60tr13c_reg_read(dev, BGT60TR13C_REG_CHIP_ID, &chip_id);
	if (ret < 0) {
		LOG_ERR("Failed to read chip ID: %d", ret);
		return ret;
	}

	if (chip_id != BGT60TR13C_CHIP_ID_VALUE) {
		LOG_ERR("Unexpected chip ID 0x%06x (expected 0x%06x)", chip_id,
			BGT60TR13C_CHIP_ID_VALUE);
		return -ENODEV;
	}

	LOG_INF("BGT60TR13C chip ID: 0x%06x", chip_id);

	/*
	 * After hardware reset MAIN defaults to a non-zero 24-bit value whose
	 * upper byte (bits[23:16]) contains hardware-enable flags for the RF
	 * frontend (LDO, PLL, TX, etc.).  We must preserve those bits.
	 * Read the current MAIN value, OR in FRAME_START (bit 0), and write
	 * the full 24-bit result back so no enable bit is inadvertently cleared.
	 */
	ret = bgt60tr13c_reg_read(dev, BGT60TR13C_REG_MAIN, &main_val);
	if (ret < 0) {
		LOG_ERR("Failed to read MAIN: %d", ret);
		return ret;
	}

	LOG_DBG("MAIN before start: 0x%06x", main_val);
	main_val |= (uint32_t)BGT60TR13C_MAIN_FRAME_START;

	ret = bgt60tr13c_reg_write(dev, BGT60TR13C_REG_MAIN, main_val);
	if (ret < 0) {
		LOG_ERR("Failed to start frame acquisition: %d", ret);
		return ret;
	}

	/* Allow at least one complete frame to arrive in the FIFO. */
	k_msleep(200);

#ifdef CONFIG_BGT60TR13C_TRIGGER
	ret = bgt60tr13c_trigger_init(dev);
	if (ret < 0) {
		LOG_ERR("Trigger init failed: %d", ret);
		return ret;
	}
#endif

	LOG_INF("BGT60TR13C initialized");
	return 0;
}

static DEVICE_API(sensor, bgt60tr13c_api) = {
	.sample_fetch = bgt60tr13c_sample_fetch,
	.channel_get = bgt60tr13c_channel_get,
#ifdef CONFIG_BGT60TR13C_TRIGGER
	.trigger_set = bgt60tr13c_trigger_set,
#endif
};

#define BGT60TR13C_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER)

#define BGT60TR13C_DEFINE(inst)                                                                    \
	static struct bgt60tr13c_data bgt60tr13c_data_##inst;                                      \
	static const struct bgt60tr13c_config bgt60tr13c_config_##inst = {                         \
		.spi = SPI_DT_SPEC_INST_GET(inst, BGT60TR13C_SPI_OPERATION),                       \
		.irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                                \
		.reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                            \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, bgt60tr13c_init, NULL, &bgt60tr13c_data_##inst,         \
				     &bgt60tr13c_config_##inst, POST_KERNEL,                       \
				     CONFIG_SENSOR_INIT_PRIORITY, &bgt60tr13c_api);

DT_INST_FOREACH_STATUS_OKAY(BGT60TR13C_DEFINE)
