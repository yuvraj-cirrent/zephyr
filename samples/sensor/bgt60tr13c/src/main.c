/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#if DT_HAS_COMPAT_STATUS_OKAY(infineon_bgt60tr13c)
#define RADAR_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(infineon_bgt60tr13c)
#else
#error "No enabled infineon,bgt60tr13c devicetree node found"
#endif

/*
 * BGT60TR13C SPI protocol: 32-bit frames, rw=0 read / rw=1 write.
 * rx[0] = GSR0 status, rx[1:3] = data[23:0].
 * MAIN bits[23:16] contain RF-frontend enable flags; must be preserved.
 */
static const struct spi_dt_spec radar_spi =
	SPI_DT_SPEC_GET(RADAR_NODE, (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER), 0);

/* Read full 24-bit register value. */
static int diag_reg_read24(uint8_t reg, uint32_t *val24)
{
	uint8_t tx[4] = {(reg << 1) | 0x00U, 0U, 0U, 0U};
	uint8_t rx[4];
	const struct spi_buf tb = {.buf = tx, .len = 4};
	const struct spi_buf rb = {.buf = rx, .len = 4};
	const struct spi_buf_set ts = {.buffers = &tb, .count = 1};
	const struct spi_buf_set rs = {.buffers = &rb, .count = 1};
	int ret = spi_transceive_dt(&radar_spi, &ts, &rs);

	if (ret == 0) {
		*val24 = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
	}
	return ret;
}

/* Write full 24-bit value to a register. */
static int diag_reg_write24(uint8_t reg, uint32_t val24)
{
	uint8_t tx[4] = {
		(reg << 1) | 0x01U,
		(uint8_t)((val24 >> 16) & 0xFFU),
		(uint8_t)((val24 >> 8) & 0xFFU),
		(uint8_t)(val24 & 0xFFU),
	};
	const struct spi_buf tb = {.buf = tx, .len = 4};
	const struct spi_buf_set ts = {.buffers = &tb, .count = 1};
	return spi_write_dt(&radar_spi, &ts);
}

/* FIFO burst read: 4-byte header {0xFF,0xC0,0x00,0x00} + count*2 data bytes. */
static int diag_fifo_burst(uint16_t *buf, uint16_t count)
{
	uint8_t tx_hdr[4] = {0xFFU, 0xC0U, 0U, 0U};
	uint8_t rx_hdr[4];
	const struct spi_buf tx_bufs[] = {
		{.buf = tx_hdr, .len = 4},
		{.buf = NULL, .len = count * 2U},
	};
	const struct spi_buf rx_bufs[] = {
		{.buf = rx_hdr, .len = 4},
		{.buf = buf, .len = count * 2U},
	};
	const struct spi_buf_set ts = {.buffers = tx_bufs, .count = 2};
	const struct spi_buf_set rs = {.buffers = rx_bufs, .count = 2};
	int ret = spi_transceive_dt(&radar_spi, &ts, &rs);

	if (ret == 0 && (rx_hdr[0] & (0x01U | 0x04U | 0x08U))) {
		printk("GSR0 error: 0x%02x\n", rx_hdr[0]);
		return -EIO;
	}
	if (ret == 0) {
		for (uint16_t i = 0; i < count; i++) {
			buf[i] = sys_be16_to_cpu(buf[i]);
		}
	}
	return ret;
}

int main(void)
{
	const struct device *radar = DEVICE_DT_GET(RADAR_NODE);
	uint32_t val24;
	uint16_t samples[8];
	int rc;

	if (!device_is_ready(radar)) {
		printk("Radar device not ready\n");
		return 0;
	}

	printk("BGT60TR13C protocol diagnostic\n");
	printk("================================\n");

	/* CHIP_ID at addr 0x02 — expect 0x000606 (digital_id=6, rf_id=6) */
	rc = diag_reg_read24(0x02, &val24);
	printk("CHIP_ID(0x02) = 0x%06x  rc=%d  (expect 0x000606)\n", val24, rc);

	/* MAIN: read full 24-bit to inspect RF-enable bits[23:16] */
	rc = diag_reg_read24(0x00, &val24);
	printk("MAIN  (0x00) = 0x%06x  rc=%d  FRAME_START=%u\n",
	       val24, rc, (unsigned)(val24 & 0x1U));

	/*
	 * If FRAME_START (bit 0) is not set, do a read-modify-write to set
	 * it while preserving bits[23:16] (RF-enable flags).  A plain write
	 * of 0x000001 would zero those flags and disable the hardware.
	 */
	if ((val24 & 0x000001UL) == 0U) {
		uint32_t new_main = val24 | 0x000001UL;

		printk("FRAME_START not set — writing MAIN=0x%06x, waiting 500 ms\n",
		       new_main);
		rc = diag_reg_write24(0x00, new_main);
		printk("  write rc=%d\n", rc);
		k_sleep(K_MSEC(500));
	}

	/* FSTAT at addr 0x5F — fill_status in bits[13:0] of 24-bit value */
	rc = diag_reg_read24(0x5F, &val24);
	printk("FSTAT (0x5F) = 0x%06x  rc=%d  fill_count=%u\n",
	       val24, rc, (unsigned)(val24 & 0x3FFFU));

	/* FIFO burst read: 8 samples */
	rc = diag_fifo_burst(samples, 8);
	printk("FIFO burst x8  rc=%d:\n", rc);
	for (int i = 0; i < 8; i++) {
		printk("  [%d] = 0x%04x (%u)\n", i, samples[i], samples[i] & 0x0FFFU);
	}

	/* FSTAT after burst — fill count should have decreased by 8 */
	rc = diag_reg_read24(0x5F, &val24);
	printk("FSTAT after burst = 0x%06x  fill_count=%u\n",
	       val24, (unsigned)(val24 & 0x3FFFU));

	/* MAIN after burst — FRAME_START auto-clears once the frame starts */
	rc = diag_reg_read24(0x00, &val24);
	printk("MAIN  after burst = 0x%06x  FRAME_START=%u\n",
	       val24, (unsigned)(val24 & 0x1U));

	printk("================================\n");

	while (1) {
		k_sleep(K_MSEC(10000));
	}

	return 0;
}
