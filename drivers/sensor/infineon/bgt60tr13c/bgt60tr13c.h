/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BGT60TR13C_H_
#define ZEPHYR_DRIVERS_SENSOR_BGT60TR13C_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

/*
 * BGT60TR13C SPI protocol: 32-bit frame (4 bytes)
 *   byte[0]    = (addr[6:0] << 1) | rw  — rw=0 read, rw=1 write
 *   bytes[1:3] = 24-bit data (MSB first)
 *   rx[0]      = GSR0 status (returned during command byte phase)
 *   rx[1:3]    = 24-bit register value (for reads; echoes old value for writes)
 */
#define BGT60TR13C_SPI_READ_BIT  0U
#define BGT60TR13C_SPI_WRITE_BIT 1U

/* Register addresses (from Infineon sensor-xensiv-bgt60trxx SDK) */
#define BGT60TR13C_REG_MAIN      0x00U
#define BGT60TR13C_REG_ADC0      0x01U
#define BGT60TR13C_REG_CHIP_ID   0x02U
#define BGT60TR13C_REG_SFCTL     0x06U /* SPI/FIFO control */
#define BGT60TR13C_REG_STAT0     0x5DU /* status: LDO_RDY, PM, CH_IDX, SH_IDX */
/* FSTAT register (0x5F): bits[13:0] = FIFO fill count (FIFO words) */
#define BGT60TR13C_REG_FSTAT     0x5FU
#define BGT60TR13C_REG_FIFO_DATA 0x60U

/* MAIN register bits (24-bit register data field) */
#define BGT60TR13C_MAIN_FRAME_START BIT(0)
#define BGT60TR13C_MAIN_RESET_MSK   0x0000EULL /* bits[3:1] */

/* Chip ID: digital_id=6 in bits[23:8], rf_id=6 in bits[7:0] of 24-bit register.
 * Full 24-bit value: 0x000606.
 */
#define BGT60TR13C_CHIP_ID_VALUE 0x000606UL

/* FSTAT fill_status occupies bits[13:0] of the 24-bit register. */
#define BGT60TR13C_FSTAT_FILL_MSK 0x3FFFU

/* FIFO: BGT60TR13C has 8192 words (2 x 12-bit samples per word).
 * We read at most FIFO_FETCH_MAX samples per sensor_sample_fetch() call.
 */
#define BGT60TR13C_FIFO_SIZE      128U
#define BGT60TR13C_FIFO_FETCH_MAX 128U

/* FIFO burst-read header (CS held low across header + data):
 *   byte[0] = 0xFF  — all-ones = burst mode indicator
 *   byte[1] = (REG_FIFO_DATA << 1) = 0xC0  — SADR with rw=0 (read)
 */
#define BGT60TR13C_BURST_HDR_B0 0xFFU
#define BGT60TR13C_BURST_HDR_B1 ((uint8_t)(BGT60TR13C_REG_FIFO_DATA << 1U))

struct bgt60tr13c_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec irq_gpio;
	struct gpio_dt_spec reset_gpio;
};

struct bgt60tr13c_data {
	uint16_t fifo_buf[BGT60TR13C_FIFO_SIZE];
	uint16_t fifo_count;

#ifdef CONFIG_BGT60TR13C_TRIGGER
	const struct device *dev;
	struct gpio_callback irq_cb;
	sensor_trigger_handler_t data_ready_handler;
	const struct sensor_trigger *data_ready_trigger;

#ifdef CONFIG_BGT60TR13C_TRIGGER_OWN_THREAD
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_BGT60TR13C_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem irq_sem;
#elif defined(CONFIG_BGT60TR13C_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_BGT60TR13C_TRIGGER */
};

#endif /* ZEPHYR_DRIVERS_SENSOR_BGT60TR13C_H_ */
