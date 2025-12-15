/**
  ******************************************************************************
  * @file    board.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <stdio.h>
#include "gpio.h"
#include "usart.h"
#include "dac.h"
#include "i2c.h"
#include "i3c.h"
#include "spi.h"
#include "sai.h"
#include "tim.h"
#include "queue.h"

#define CMD_MAX_SIZE 64
#define LOG_MESSAGE(fmt, ...) fprintf(stdout, fmt "\n",##__VA_ARGS__)
#define DEBUG_MESSAGE(fmt, ...) fprintf(stderr, fmt "\n",##__VA_ARGS__)
#define ATTEMPT_WITH_RETRY(call, result, until, max_retry) do { \
                                                            int32_t retry = max_retry; \
                                                            do { \
                                                              result = call; \
                                                              retry--; \
                                                            } while (!(until) && retry > 0); \
                                                           } while (0);

enum led { ORANGE, GREEN, RED };

struct i2c_conf {
  uint8_t sa0;
  uint8_t dev_addr;
};

enum wire { WIRE_4, WIRE_3 };

enum spi_freq {
  SPI_FREQ_1_MHZ = SPI_BAUDRATEPRESCALER_256,
  SPI_FREQ_2_MHZ = SPI_BAUDRATEPRESCALER_128,
  SPI_FREQ_4_MHZ = SPI_BAUDRATEPRESCALER_64,
  SPI_FREQ_8_MHZ = SPI_BAUDRATEPRESCALER_32,
  SPI_FREQ_16_MHZ = SPI_BAUDRATEPRESCALER_16
};

struct spi_conf {
  enum wire wire;
};

struct i3c_conf {
  uint8_t sa0;
  uint8_t static_address;
  uint8_t dynamic_address;
  uint32_t bus_frequency;
};

enum sai_rate { SAI_RATE_8KHZ, SAI_RATE_16KHZ };

struct cmd_parser {
  uint8_t buf[CMD_MAX_SIZE];
  uint32_t cnt;
  uint8_t collect;
};

void set_vdd(float voltage);
void set_vddio(float voltage);
void set_range_100x(uint8_t en);
void delay(uint32_t msec);
uint32_t systick(void);

void i2c_init(struct i2c_conf *i2c_conf);
void spi_init(struct spi_conf *spi_conf);
void spi_set_freq(enum spi_freq freq);
void i3c_init(struct i3c_conf *i3c_conf);
void get_i3c_ibi_payload(uint32_t *tgt_addr, uint32_t *ibi_payload_size, uint8_t *payload);
void spi_aux_init(struct spi_conf *spi_conf);
void usb_cdc_flush(void);
void usb_cdc_write(void *buff, uint16_t len);

void led_on(enum led led);
void led_off(enum led led);
void led_toggle(enum led led);

void write(uint8_t addr, uint8_t val);
void read(uint8_t addr, void *val, uint16_t len);
void write_aux(uint8_t addr, uint8_t val);
void read_aux(uint8_t addr, void *val, uint16_t len);

void sai_init(enum sai_rate sai_rate, uint8_t n_slots);
void sai_start(void *buff, uint16_t size);
void sai_stop(void);

void tim_evt_start(uint32_t period_us);
void tim_evt_stop(void);
void delay_us(uint16_t time_us);
void meas_start(void);
uint32_t meas_stop(void);

enum button { SW1, SW2 };
enum button_state { BUTTON_RELEASED, BUTTON_PRESSED };
enum button_state button_get_state(enum button button);

void uart_cmd_listen_start(void);
void cmd_reset(struct cmd_parser *cmd);
void cmd_parse(struct cmd_parser *cmd, uint8_t cmd_char);
uint32_t cmd_fifo_enqueue(uint8_t *cmd);
uint8_t cmd_fifo_dequeue(uint8_t *cmd_buf, uint32_t cmd_buf_size);

#endif /* BOARD_H */
