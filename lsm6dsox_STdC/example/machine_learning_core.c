/*
 ******************************************************************************
 * @file    machine_learning_core.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
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
 *                                       - I2C(Default)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` is required.
 *
 * The purpose of this example is to show how use the device Machine Learning
 * Core (MLC) starting from an ".ucf" file generated with the "Machine
 * Learning Core" tool.
 *
 * Below is showed an ".ucf" file generated with the "Machine Learning Core"
 * tool available in the Unico GUI.
 * For more information about Machine Learning Core tool please refer
 * to AN5259 "LSM6DSOX: Machine Learning Core".
 *
 *   --->  Filename: LSM6DSOX_VibrationMonitoring.ucf <----
 * -- MLC GUI v1.2.0.0 Beta, LSM6DSOX
 *
 * Ac 10 00
 * Ac 11 00
 * Ac 01 80
 * Ac 17 10
 * WAIT 50 ms
 * Ac 05 00
 * Ac 17 00
 * Ac 01 00
 * Ac 10 A0
 * Ac 11 A0
 * Ac 10 00
 * Ac 11 00
 * Ac 01 80
 * Ac 17 40
 * Ac 02 11
 * Ac 08 EA
 * Ac 09 46
 * Ac 02 11
 * Ac 08 EB
 * Ac 09 03
 * Ac 02 11
 * Ac 08 EC
 * Ac 09 50
 * Ac 02 11
 * Ac 08 ED
 * Ac 09 03
 * Ac 02 11
 * Ac 08 EE
 * Ac 09 00
 * Ac 02 11
 * Ac 08 EF
 * Ac 09 00
 * Ac 02 11
 * Ac 08 F0
 * Ac 09 0A
 * Ac 02 11
 * Ac 08 F2
 * Ac 09 10
 * Ac 02 11
 * Ac 08 FA
 * Ac 09 3C
 * Ac 02 11
 * Ac 08 FB
 * Ac 09 03
 * Ac 02 11
 * Ac 08 FC
 * Ac 09 52
 * Ac 02 11
 * Ac 08 FD
 * Ac 09 03
 * Ac 02 11
 * Ac 08 FE
 * Ac 09 5E
 * Ac 02 11
 * Ac 08 FF
 * Ac 09 03
 * Ac 02 31
 * Ac 08 3C
 * Ac 09 3F
 * Ac 02 31
 * Ac 08 3D
 * Ac 09 00
 * Ac 02 31
 * Ac 08 3E
 * Ac 09 03
 * Ac 02 31
 * Ac 08 3F
 * Ac 09 94
 * Ac 02 31
 * Ac 08 40
 * Ac 09 00
 * Ac 02 31
 * Ac 08 41
 * Ac 09 FC
 * Ac 02 31
 * Ac 08 42
 * Ac 09 00
 * Ac 02 31
 * Ac 08 43
 * Ac 09 7C
 * Ac 02 31
 * Ac 08 44
 * Ac 09 1F
 * Ac 02 31
 * Ac 08 45
 * Ac 09 00
 * Ac 02 31
 * Ac 08 52
 * Ac 09 00
 * Ac 02 31
 * Ac 08 53
 * Ac 09 00
 * Ac 02 31
 * Ac 08 54
 * Ac 09 00
 * Ac 02 31
 * Ac 08 55
 * Ac 09 00
 * Ac 02 31
 * Ac 08 56
 * Ac 09 00
 * Ac 02 31
 * Ac 08 57
 * Ac 09 00
 * Ac 02 31
 * Ac 08 58
 * Ac 09 00
 * Ac 02 31
 * Ac 08 59
 * Ac 09 00
 * Ac 02 31
 * Ac 08 5A
 * Ac 09 00
 * Ac 02 31
 * Ac 08 5B
 * Ac 09 00
 * Ac 02 31
 * Ac 08 5C
 * Ac 09 00
 * Ac 01 00
 * Ac 12 00
 * Ac 01 80
 * Ac 17 40
 * Ac 02 31
 * Ac 08 5E
 * Ac 09 AE
 * Ac 02 31
 * Ac 08 5F
 * Ac 09 27
 * Ac 02 31
 * Ac 08 60
 * Ac 09 10
 * Ac 02 31
 * Ac 08 61
 * Ac 09 C0
 * Ac 09 00
 * Ac 09 00
 * Ac 02 31
 * Ac 08 62
 * Ac 09 00
 * Ac 02 31
 * Ac 08 63
 * Ac 09 3E
 * Ac 02 31
 * Ac 08 64
 * Ac 09 21
 * Ac 02 31
 * Ac 08 65
 * Ac 09 E0
 * Ac 09 00
 * Ac 09 00
 * Ac 01 80
 * Ac 17 00
 * Ac 04 00
 * Ac 05 10
 * Ac 02 01
 * Ac 01 00
 * Ac 12 04
 * Ac 01 80
 * Ac 60 15
 * Ac 01 00
 * Ac 10 28
 * Ac 11 00
 *
 *   ---> End File <---
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <lsm6dsox_reg.h>
#include "gpio.h"
#include "i2c.h"
#include "usart.h"

/* This structure is used to import one line of.ucf file*/
typedef struct {
  enum { REG, WAIT} type;
  uint8_t reg;
  uint8_t data;
} ucf_line_t;

/* Decision tree: vibration_monitoring - all the .ucf file*/
const ucf_line_t lsm6dsox_mlc_vibration_monitoring[] = {
 { .type = REG,  .reg = 0x10, .data = 0x00 },
 { .type = REG,  .reg = 0x11, .data = 0x00 },
 { .type = REG,  .reg = 0x01, .data = 0x80 },
 { .type = REG,  .reg = 0x17, .data = 0x10 },
 { .type = WAIT, .reg = 0x00, .data = 50   }, //ms
 { .type = REG,  .reg = 0x05, .data = 0x00 },
 { .type = REG,  .reg = 0x17, .data = 0x00 },
 { .type = REG,  .reg = 0x01, .data = 0x00 },
 { .type = REG,  .reg = 0x10, .data = 0xA0 },
 { .type = REG,  .reg = 0x11, .data = 0xA0 },
 { .type = REG,  .reg = 0x10, .data = 0x00 },
 { .type = REG,  .reg = 0x11, .data = 0x00 },
 { .type = REG,  .reg = 0x01, .data = 0x80 },
 { .type = REG,  .reg = 0x17, .data = 0x40 },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xEA },
 { .type = REG,  .reg = 0x09, .data = 0x46 },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xEB },
 { .type = REG,  .reg = 0x09, .data = 0x03 },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xEC },
 { .type = REG,  .reg = 0x09, .data = 0x50 },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xED },
 { .type = REG,  .reg = 0x09, .data = 0x03 },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xEE },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xEF },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xF0 },
 { .type = REG,  .reg = 0x09, .data = 0x0A },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xF2 },
 { .type = REG,  .reg = 0x09, .data = 0x10 },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xFA },
 { .type = REG,  .reg = 0x09, .data = 0x3C },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xFB },
 { .type = REG,  .reg = 0x09, .data = 0x03 },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xFC },
 { .type = REG,  .reg = 0x09, .data = 0x52 },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xFD },
 { .type = REG,  .reg = 0x09, .data = 0x03 },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xFE },
 { .type = REG,  .reg = 0x09, .data = 0x5E },
 { .type = REG,  .reg = 0x02, .data = 0x11 },
 { .type = REG,  .reg = 0x08, .data = 0xFF },
 { .type = REG,  .reg = 0x09, .data = 0x03 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x3C },
 { .type = REG,  .reg = 0x09, .data = 0x3F },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x3D },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x3E },
 { .type = REG,  .reg = 0x09, .data = 0x03 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x3F },
 { .type = REG,  .reg = 0x09, .data = 0x94 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x40 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x41 },
 { .type = REG,  .reg = 0x09, .data = 0xFC },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x42 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x43 },
 { .type = REG,  .reg = 0x09, .data = 0x7C },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x44 },
 { .type = REG,  .reg = 0x09, .data = 0x1F },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x45 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x52 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x53 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x54 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x55 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x56 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x57 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x58 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x59 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x5A },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x5B },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x5C },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x01, .data = 0x00 },
 { .type = REG,  .reg = 0x12, .data = 0x00 },
 { .type = REG,  .reg = 0x01, .data = 0x80 },
 { .type = REG,  .reg = 0x17, .data = 0x40 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x5E },
 { .type = REG,  .reg = 0x09, .data = 0xAE },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x5F },
 { .type = REG,  .reg = 0x09, .data = 0x27 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x60 },
 { .type = REG,  .reg = 0x09, .data = 0x10 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x61 },
 { .type = REG,  .reg = 0x09, .data = 0xC0 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x62 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x63 },
 { .type = REG,  .reg = 0x09, .data = 0x3E },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x64 },
 { .type = REG,  .reg = 0x09, .data = 0x21 },
 { .type = REG,  .reg = 0x02, .data = 0x31 },
 { .type = REG,  .reg = 0x08, .data = 0x65 },
 { .type = REG,  .reg = 0x09, .data = 0xE0 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x09, .data = 0x00 },
 { .type = REG,  .reg = 0x01, .data = 0x80 },
 { .type = REG,  .reg = 0x17, .data = 0x00 },
 { .type = REG,  .reg = 0x04, .data = 0x00 },
 { .type = REG,  .reg = 0x05, .data = 0x10 },
 { .type = REG,  .reg = 0x02, .data = 0x01 },
 { .type = REG,  .reg = 0x01, .data = 0x00 },
 { .type = REG,  .reg = 0x12, .data = 0x04 },
 { .type = REG,  .reg = 0x01, .data = 0x80 },
 { .type = REG,  .reg = 0x60, .data = 0x15 },
 { .type = REG,  .reg = 0x01, .data = 0x00 },
 { .type = REG,  .reg = 0x10, .data = 0x28 },
 { .type = REG,  .reg = 0x11, .data = 0x00 },
};

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

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

static void tx_com( uint8_t *tx_buffer, uint16_t len );

/* Main Example --------------------------------------------------------------*/
void lsm6dsox_mlc(void)
{
  /* Variable declaration */
  lsm6dsox_ctx_t              dev_ctx;
  lsm6dsox_pin_int1_route_t   pin_int1_route;
  lsm6dsox_all_sources_t      status;
  uint8_t                     i, mlc_out[8];

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg  = platform_read;
  dev_ctx.handle    = &hi2c1;

  /* Check device ID */
  lsm6dsox_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DSOX_ID)
    while(1);

  /* Restore default configuration (not FSM) */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Start Machine Learning Core configuration */  
  for ( i = 0; i < (sizeof(lsm6dsox_mlc_vibration_monitoring) / 
                    sizeof(ucf_line_t) ); i++ ){
                     
    if ( lsm6dsox_mlc_vibration_monitoring[i].type == REG ){ 
      
      lsm6dsox_write_reg(&dev_ctx, lsm6dsox_mlc_vibration_monitoring[i].reg, 
                         (uint8_t*)&lsm6dsox_mlc_vibration_monitoring[i].data, 1); 
      
    }
    else {
      
      HAL_Delay(lsm6dsox_mlc_vibration_monitoring[i].data); 
      
    }
    
  }
  /* End Machine Learning Core configuration */

  /* Turn off Sensors */ 
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_OFF);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_OFF);  
  
  /* Disable I3C interface */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);

  /* Enable Block Data Update */
  lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_4g);
  lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_2000dps);

  /* Route signals on interrupt pin 1 */
  lsm6dsox_pin_int1_route_get(&dev_ctx, &pin_int1_route);
  pin_int1_route.mlc_int1.int1_mlc1 = PROPERTY_ENABLE;
  lsm6dsox_pin_int1_route_set(&dev_ctx, &pin_int1_route);

  /* Configure interrupt pin mode notification */
  lsm6dsox_int_notification_set(&dev_ctx, LSM6DSOX_BASE_PULSED_EMB_LATCHED);

  /* Set Output Data Rate. 
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate. 
   */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_26Hz);
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_OFF); 
  
  /* Main loop */
  while(1)
  {
    /* Read interrupt source registers in polling mode (no int) */
    lsm6dsox_all_sources_get(&dev_ctx, &status);
    if (status.mlc_status.is_mlc1){
      
      lsm6dsox_mlc_out_get(&dev_ctx, mlc_out);
      sprintf((char*)tx_buffer, "Detect MLC interrupt code: %02X\r\n",
              mlc_out[0]);
        
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }
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
    HAL_I2C_Mem_Write(handle, LSM6DSOX_I2C_ADD_H, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
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
    HAL_I2C_Mem_Read(handle, LSM6DSOX_I2C_ADD_H, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to trasmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

