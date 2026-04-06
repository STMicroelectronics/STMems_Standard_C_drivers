/**
  ******************************************************************************
  * @file    board.c
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
#include "board.h"
#include <stdio.h>
#include "gpio.h"
#include "usart.h"
#include "dac.h"
#include "i2c.h"
#include "spi.h"
#include "i3c.h"
#include "sai.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include "queue.h"

extern void MX_SPI1_Init_3W(void);
extern void MX_SPI6_Init_3W(void);

static msg_queue usb_queue = {{0}, 0, 0};
static uint8_t usb_tmp_buf[MSG_QUEUE_MAX_CAPACITY];

static msg_queue cmd_queue = {{0}, 0, 0};

void usb_cdc_flush(void)
{
  if (!msg_queue_is_empty(&usb_queue)) {
    uint32_t size = (uint16_t)msg_queue_get_size(&usb_queue);
    /* get the whole content of the queue */
    msg_queue_to_arr(&usb_queue, usb_tmp_buf, sizeof(usb_tmp_buf));
    uint8_t res = CDC_Transmit_FS(usb_tmp_buf, (uint16_t)size);
    if (res == USBD_OK) {
      /* if the transmission succeed, flush the whole content of the queue */
      msg_queue_flush(&usb_queue);
    }
  }
}

void usb_cdc_write(void *buff, uint16_t len)
{
  CDC_Transmit_FS(buff, len);
}

void set_vdd(float voltage)
{
  uint32_t val = (uint32_t)(4095.0f * voltage / 3.6f);
  HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, val);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
}

void set_vddio(float voltage)
{
  uint32_t val = (uint32_t)(4095.0f * voltage / 3.6f);
  HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, val);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_2);
}

void set_range_100x(uint8_t en)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  if (en) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
  }
}

void delay(uint32_t msec)
{
  HAL_Delay(msec);
}

uint32_t systick(void)
{
  return HAL_GetTick();
}

enum { NOT_VALID, I2C, I3C, SPI };

static uint8_t interface;

void spi_init(struct spi_conf *spi_conf)
{
  switch (spi_conf->wire) {
  case WIRE_4:
    MX_SPI1_Init();
    interface = SPI;
    break;
  case WIRE_3:
    MX_SPI1_3W_Init();
    interface = SPI;
    break;
  case WIRE_4_EXT:
    MX_SPI1_Ext_Init();
    interface = SPI;
    break;
  case WIRE_3_EXT:
    MX_SPI1_3W_Ext_Init();
    interface = SPI;
    break;
  default:
    break;
  }
}


extern SPI_HandleTypeDef hspi1;
void spi_set_freq(enum spi_freq freq)
{
  hspi1.Init.BaudRatePrescaler = freq;
  HAL_SPI_Init(&hspi1);
}

void write(uint8_t addr, uint8_t val)
{
  uint8_t buff[2] = { addr, val };

  switch (interface) {
  case SPI:
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, buff, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    break;
  }
}

void read(uint8_t addr, void *val, uint16_t len)
{
  switch (interface) {
  case SPI:
    addr |= 0x80;
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, val, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
    break;
  }
}

void spi_aux_init(struct spi_conf *spi_conf)
{
  /* disable transceiver */
  HAL_GPIO_WritePin(SAI_MCLK_FS_OE_GPIO_Port, SAI_MCLK_FS_OE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SAI_SCK_SD_OE_GPIO_Port, SAI_SCK_SD_OE_Pin, GPIO_PIN_SET);

  /* configure DIL24_SPIx6_CS pin as output */
  HAL_GPIO_WritePin(DIL24_SPIx6_CS_GPIO_Port, DIL24_SPIx6_CS_Pin, GPIO_PIN_SET);
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = DIL24_SPIx6_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIL24_SPIx6_CS_GPIO_Port, &GPIO_InitStruct);

  switch (spi_conf->wire) {
  case WIRE_4:
    MX_SPI6_Init();
    break;
  case WIRE_3:
    MX_SPI6_Init_3W();
    break;
  default:
    break;
  }
}

void write_aux(uint8_t addr, uint8_t val)
{
  uint8_t buff[2] = { addr, val };

  HAL_GPIO_WritePin(DIL24_SPIx6_CS_GPIO_Port, DIL24_SPIx6_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi6, buff, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(DIL24_SPIx6_CS_GPIO_Port, DIL24_SPIx6_CS_Pin, GPIO_PIN_SET);
}

void read_aux(uint8_t addr, void *val, uint16_t len)
{
  addr |= 0x80;
  HAL_GPIO_WritePin(DIL24_SPIx6_CS_GPIO_Port, DIL24_SPIx6_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi6, &addr, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi6, val, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(DIL24_SPIx6_CS_GPIO_Port, DIL24_SPIx6_CS_Pin, GPIO_PIN_SET);
}

void sai_init(enum sai_rate sai_rate, uint8_t n_slots)
{
  /* enable transceiver */
  HAL_GPIO_WritePin(SAI_MCLK_FS_OE_GPIO_Port, SAI_MCLK_FS_OE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SAI_SCK_SD_OE_GPIO_Port, SAI_SCK_SD_OE_Pin, GPIO_PIN_RESET);

  /* configure SAI1_MCLK_A pin as output set to ground */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* configure transceiver direction */
  HAL_GPIO_WritePin(SAI_MCLK_DIR_GPIO_Port, SAI_MCLK_DIR_Pin, GPIO_PIN_SET);

  /* configure DIL24_SPIx6_CS pin as input */
  GPIO_InitStruct.Pin = DIL24_SPIx6_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIL24_SPIx6_CS_GPIO_Port, &GPIO_InitStruct);

  MX_SAI1_Init();

  switch (sai_rate) {
  case SAI_RATE_8KHZ:
    hsai_BlockB1.FrameInit.FrameLength = 256;
    break;
  case SAI_RATE_16KHZ:
    hsai_BlockB1.FrameInit.FrameLength = 128;
    break;
  }

  switch (n_slots) {
  case 1:
    hsai_BlockB1.SlotInit.SlotNumber = 1;
    hsai_BlockB1.SlotInit.SlotActive = 0x00000001;
    break;
  case 2:
    hsai_BlockB1.SlotInit.SlotNumber = 2;
    hsai_BlockB1.SlotInit.SlotActive = 0x00000003;
    break;
  case 3:
    hsai_BlockB1.SlotInit.SlotNumber = 3;
    hsai_BlockB1.SlotInit.SlotActive = 0x00000007;
    break;
  default:
    hsai_BlockB1.SlotInit.SlotNumber = 3;
    hsai_BlockB1.SlotInit.SlotActive = 0x00000007;
    break;
  }

  HAL_SAI_Init(&hsai_BlockB1);
}

void sai_start(void *buff, uint16_t size)
{
  HAL_SAI_Receive_DMA(&hsai_BlockB1, buff, size);
}

void sai_stop(void)
{
  HAL_SAI_Abort(&hsai_BlockB1);
}

__weak void sai_rx_cplt_cb(void)
{
  printf("%u %s\n", HAL_GetTick(), __func__);

  return;
}


__weak void sai_rx_hcplt_cb(void)
{
  printf("%u %s\n", HAL_GetTick(), __func__);

  return;
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  sai_rx_cplt_cb();
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  sai_rx_hcplt_cb();
}

void tim_evt_start(uint32_t period_us)
{
  MX_TIM2_Init();
  uint32_t autoreload = period_us - 1;

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim2, autoreload);
  HAL_TIM_Base_Start_IT(&htim2);
}

void tim_evt_stop(void)
{
  HAL_TIM_Base_Stop_IT(&htim2);
}

static volatile uint8_t delay_us_int;

void delay_us(uint16_t time_us)
{
  __HAL_TIM_SET_COUNTER(&htim6, 0);
  while (__HAL_TIM_GET_COUNTER(&htim6) < time_us);

  return;
}

static uint32_t _meas_start;
void meas_start(void)
{
  _meas_start = __HAL_TIM_GET_COUNTER(&htim5);
}

uint32_t meas_stop(void)
{
  uint32_t _meas_stop = __HAL_TIM_GET_COUNTER(&htim5);

  return _meas_stop - _meas_start;
}

__weak void tim_evt_callback(void)
{
  printf("%u %s\n", HAL_GetTick(), __func__);

  return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    tim_evt_callback();
  }

  return;
}

void led_on(enum led led)
{
  switch (led) {
  case ORANGE:
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    break;
  case GREEN:
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    break;
  case RED:
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    break;
  }
}

void led_off(enum led led)
{
  switch (led) {
  case ORANGE:
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    break;
  case GREEN:
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    break;
  case RED:
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    break;
  }
}

void led_toggle(enum led led)
{
  switch (led) {
  case ORANGE:
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    break;
  case GREEN:
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    break;
  case RED:
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    break;
  }
}

enum button_state button_get_state(enum button button)
{
  enum button_state ret = BUTTON_RELEASED;

  switch (button) {
  case SW1:
    ret = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_SET ? BUTTON_PRESSED : BUTTON_RELEASED;
    break;
  case SW2:
    ret = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_SET ? BUTTON_PRESSED : BUTTON_RELEASED;
    break;
  }

  return ret;
}

uint32_t cmd_fifo_enqueue(uint8_t *cmd)
{
  // return the number of enqueued characters
  return msg_enqueue(&cmd_queue, cmd, strlen((const char *)cmd) + 1);
}

uint8_t cmd_fifo_dequeue(uint8_t *cmd_buf, uint32_t cmd_buf_size)
{
  uint8_t res = 0;
  if (!msg_queue_is_empty(&cmd_queue)) {
    str_msg_dequeue(&cmd_queue, cmd_buf, cmd_buf_size);
    res = 1;
  }

  return res;
}

void cmd_reset(struct cmd_parser *cmd)
{
  memset(cmd->buf, 0, CMD_MAX_SIZE);
  cmd->cnt = 0;
  cmd->collect = 0;
}

void cmd_parse(struct cmd_parser *cmd, uint8_t cmd_char)
{
  if (cmd->collect) {
    if (cmd_char == '\r' || cmd_char == '\n') {
      cmd->buf[cmd->cnt] = '\0';
      // dummy lock on command queue
      __disable_irq();
      cmd_fifo_enqueue(cmd->buf);
      __enable_irq();
      cmd->collect = 0;
      cmd->cnt = 0;
    } else if (cmd_char == '*') {
      cmd->cnt = 0;
    } else {
      cmd->buf[cmd->cnt] = cmd_char;
      cmd->cnt = (cmd->cnt + 1) % CMD_MAX_SIZE;
    }
  } else if (cmd_char == '*') {
    cmd->collect = 1;
  }
}

static struct cmd_parser uart_cmd;
static uint8_t uart_cmd_char;

void uart_cmd_listen_start(void)
{
  cmd_reset(&uart_cmd);
  HAL_UART_Receive_IT(&huart2, &uart_cmd_char, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  cmd_parse(&uart_cmd, uart_cmd_char);
  HAL_UART_Receive_IT(&huart2, &uart_cmd_char, 1);
}

