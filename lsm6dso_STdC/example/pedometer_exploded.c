/*
 ******************************************************************************
 * @file    pedometer.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to configure the step counter.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A3
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A3 - Host side: UART(COM) to USB bridge
 *                                       - I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: 'platform_write', 'platform_read' and
 * 'tx_com'  is required.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <lsm6dso_reg.h>
#include "gpio.h"
#include "i2c.h"
#include "usart.h"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
lsm6dso_page_address_t  page_address;
lsm6dso_emb_func_en_a_t emb_func_en_a;
lsm6dso_emb_func_en_b_t emb_func_en_b;
lsm6dso_pedo_cmd_reg_t pedo_cmd_reg;
lsm6dso_page_rw_t page_rw;
lsm6dso_page_sel_t page_sel;
static uint16_t steps, i;
static uint8_t whoamI, rst;
uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

void tx_com( uint8_t *tx_buffer, uint16_t len );

/* Main Example --------------------------------------------------------------*/
void lsm6dso_pedometer(void)
{
  lsm6dso_reg_t reg;
  lsm6dso_ctx_t ag_ctx;

  /* Initialize driver interface */
  ag_ctx.write_reg = platform_write;
  ag_ctx.read_reg = platform_read;
  ag_ctx.handle = &hi2c1;

  /* Check device ID */
  lsm6dso_device_id_get(&ag_ctx, &whoamI);
  if (whoamI != LSM6DSO_ID)
    while(1);
 
  /* Restore default configuration */
  lsm6dso_reset_set(&ag_ctx, PROPERTY_ENABLE);
  do {
    lsm6dso_reset_get(&ag_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dso_i3c_disable_set(&ag_ctx, LSM6DSO_I3C_DISABLE);

  /* Set XL full scale */
  lsm6dso_xl_full_scale_set(&ag_ctx, LSM6DSO_2g);

  /* Enable Block Data Update */
  lsm6dso_block_data_update_set(&ag_ctx, PROPERTY_ENABLE);

  /* Enable xl sensor */
  lsm6dso_xl_data_rate_set(&ag_ctx, LSM6DSO_XL_ODR_26Hz); 
 
  /* Reset steps of pedometer */
  lsm6dso_steps_reset(&ag_ctx);

 
  for ( i = 0; i < 500; i++){

    /* Disable Accelerometer */
    lsm6dso_read_reg(&ag_ctx, LSM6DSO_CTRL1_XL, &reg.byte, 1);
    reg. ctrl1_xl.odr_xl = LSM6DSO_XL_ODR_OFF;
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_CTRL1_XL, &reg.byte, 1);

    /* Disable Gyro */
    lsm6dso_read_reg(&ag_ctx, LSM6DSO_CTRL2_G, &reg.byte, 1);
    reg. ctrl2_g.odr_g = LSM6DSO_GY_ODR_OFF;
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_CTRL2_G, &reg.byte, 1);

    /* Enable pedometer */
    /* lsm6dso_ln_pg_read_byte(ctx, LSM6DSO_PEDO_CMD_REG, (uint8_t*)&pedo_cmd_reg); */
    lsm6dso_mem_bank_set(&ag_ctx, LSM6DSO_EMBEDDED_FUNC_BANK);
   
    lsm6dso_read_reg(&ag_ctx, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);
    page_rw.page_rw = 0x01; /* page_read enable*/
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);

    lsm6dso_read_reg(&ag_ctx, LSM6DSO_PAGE_SEL, (uint8_t*) &page_sel, 1);
    page_sel.page_sel = ((uint8_t)(LSM6DSO_PEDO_CMD_REG >> 8) & 0x0FU);
    page_sel.not_used_01 = 1;
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_PAGE_SEL, (uint8_t*) &page_sel, 1);

    page_address.page_addr = (uint8_t)LSM6DSO_PEDO_CMD_REG & 0x00FFU;
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_PAGE_ADDRESS, (uint8_t*)&page_address, 1);

    lsm6dso_read_reg(&ag_ctx, LSM6DSO_PAGE_VALUE, (uint8_t*)&pedo_cmd_reg, 1);

    lsm6dso_read_reg(&ag_ctx, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);

    page_rw.page_rw = 0x00; /* page_read disable */
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);

    //lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);
    /* END lsm6dso_ln_pg_read_byte(ctx, LSM6DSO_PEDO_CMD_REG, (uint8_t*)&pedo_cmd_reg); */

    //lsm6dso_mem_bank_set(&ag_ctx, LSM6DSO_EMBEDDED_FUNC_BANK);

    lsm6dso_read_reg(&ag_ctx, LSM6DSO_EMB_FUNC_EN_A, (uint8_t*)&emb_func_en_a, 1);
    lsm6dso_read_reg(&ag_ctx, LSM6DSO_EMB_FUNC_EN_B, (uint8_t*)&emb_func_en_b, 1);

    emb_func_en_a.pedo_en = 1;
    emb_func_en_b.pedo_adv_en = 1;
    pedo_cmd_reg.fp_rejection_en = 1;
    pedo_cmd_reg.ad_det_en = 1;

    lsm6dso_write_reg(&ag_ctx, LSM6DSO_EMB_FUNC_EN_A, (uint8_t*)&emb_func_en_a, 1);
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_EMB_FUNC_EN_B, (uint8_t*)&emb_func_en_b, 1);
    
    //lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);
    
    /* lsm6dso_ln_pg_write_byte(ctx, LSM6DSO_PEDO_CMD_REG,(uint8_t*)&pedo_cmd_reg); */
    //lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);

    lsm6dso_read_reg(&ag_ctx, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);

    page_rw.page_rw = 0x02; /* page_write enable */
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);

    lsm6dso_read_reg(&ag_ctx, LSM6DSO_PAGE_SEL, (uint8_t*) &page_sel, 1);

    page_sel.page_sel = ((uint8_t)(LSM6DSO_PEDO_CMD_REG >> 8) & 0x0FU);
    page_sel.not_used_01 = 1;
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_PAGE_SEL, (uint8_t*) &page_sel, 1);

    page_address.page_addr = (uint8_t)LSM6DSO_PEDO_CMD_REG & 0xFFU;
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_PAGE_ADDRESS, (uint8_t*)&page_address, 1);

    lsm6dso_write_reg(&ag_ctx, LSM6DSO_PAGE_VALUE, (uint8_t*)&pedo_cmd_reg, 1);

    lsm6dso_read_reg(&ag_ctx, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);

    page_rw.page_rw = 0x00; /* page_write disable */
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);

    lsm6dso_mem_bank_set(&ag_ctx, LSM6DSO_USER_BANK);

    /* END lsm6dso_ln_pg_write_byte(ctx, LSM6DSO_PEDO_CMD_REG,(uint8_t*)&pedo_cmd_reg); */ 
    /* END Enable pedometer */
 
    /* Enable Accelerometer */
    lsm6dso_read_reg(&ag_ctx, LSM6DSO_CTRL1_XL, &reg.byte, 1);
    reg. ctrl1_xl.odr_xl = LSM6DSO_XL_ODR_26Hz;
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_CTRL1_XL, &reg.byte, 1);

    /* Enable Gyro */
    lsm6dso_read_reg(&ag_ctx, LSM6DSO_CTRL2_G, &reg.byte, 1);
    reg. ctrl2_g.odr_g = LSM6DSO_GY_ODR_26Hz;
    lsm6dso_write_reg(&ag_ctx, LSM6DSO_CTRL2_G, &reg.byte, 1);

    HAL_Delay(50); //50 ms
  }
 
  sprintf((char*)tx_buffer, "START:%d\r\n", i);
  tx_com(tx_buffer, strlen((char const*)tx_buffer)); 
 
  while(1) {
      /* Read steps */
      lsm6dso_number_of_steps_get(&ag_ctx, (uint8_t*)&steps);
      sprintf((char*)tx_buffer, "steps :%d\r\n", steps);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
      HAL_Delay(1000);
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, LSM6DSO_I2C_ADD_H, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
#ifdef STEVAL_MKI109V3
  else if (handle == &hspi2)
  {
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }
#endif
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  return HAL_I2C_Mem_Read(handle, LSM6DSO_I2C_ADD_H, reg,
                          I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to trasmit
 * @param  len           number of byte to send
 *
 */
void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

