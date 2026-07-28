/*
 ******************************************************************************
 * @file  nmh1000_magnetic_field_detect.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows an example of how to configure the NMH1000 in
 *      Auto-Mode and handle the OUT interrupt generated when a magnetic
 *      field is detected. Upon each interrupt occurrence, the main
 *      program reads the magnetic field value and prints it to the
 *      serial terminal.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 * opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

 /*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - NUCLEO_F401RE + FRDM-STBI-NMH1000
 * - DISCOVERY_SPC584B + FRDM-STBI-NMH1000
 *
 * Used interfaces:
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *          - Sensor side: I2C(Default)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *          - Sensor side: I2C(Default)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

 /* STMicroelectronics evaluation boards definition
  *
  * Please uncomment ONLY the evaluation boards in use.
  * If a different hardware is used please comment all
  * following target board and redefine yours.
  */

//#define NUCLEO_F401RE  /* little endian */
//#define SPC584B_DIS    /* big endian */


/* ATTENTION: By default the driver is little endian. If you need switch
 *      to big endian please see "Endianness definitions" in the
 *      header file of the driver (_reg.h).
 */

#if defined(NUCLEO_F401RE)
 /* NUCLEO_F401RE: Define communication interface */
#define SENSOR_BUS hi2c1

#elif defined(SPC584B_DIS)
 /* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "nmh1000_reg.h"

#if defined(NUCLEO_F401RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(SPC584B_DIS)
#include "components.h"

#endif

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

#define BOOT_TIME     100

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t tx_buffer[1000];
stmdev_ctx_t dev_ctx;
/* USER CODE END PV */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile bool nmh1000_out_flag = false;
volatile uint8_t count = 0;
/* USER CODE END 0 */
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void* handle, uint8_t reg, const uint8_t* bufp,
  uint16_t len);
static int32_t platform_read(void* handle, uint8_t reg, uint8_t* bufp,
  uint16_t len);
static void tx_com(uint8_t* tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);


/* NMH1000 Magnetic Field Detection ISR */
void nmh1000_magfield_detection_irq_handler(void)
{
  /* Set flag to indicate Sensor has detected magnetic field */
  nmh1000_out_flag = true;

}

/* Main Example --------------------------------------------------------------*/
/* Initialize NMH1000 Device */
int32_t nmh1000_init_example(void)
{
  uint8_t who;
  nmh1000_user_odr_val_t val;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  nmh1000_reset(&dev_ctx);
  nmh1000_device_id_get(&dev_ctx, &who);

  nmh1000_status_t status;
  nmh1000_status_get(&dev_ctx, &status);

  if (NMH1000_WHO_AM_I_VAL == who)
  {
    snprintf((char*)tx_buffer, sizeof(tx_buffer),
      "Successfully Initialized Ayomani (Generic Open Market) with WHO_AM_I = 0x%X\r\n", who);
    tx_com(tx_buffer, strlen((char const*)tx_buffer));
  }
  else if (NMH1000_WHO_AM_I_ERROR == who) /* WHO_AM_I = 0x0 */
  {
    snprintf((char*)tx_buffer, sizeof(tx_buffer),
      "Error Condition - WHO_AM_I = 0x%X\r\n", who);
    tx_com(tx_buffer, strlen((char const*)tx_buffer));
    return -1;
  }
  else if (NMH1000_WHO_AM_I_NPROG == who) /* WHO_AM_I = 0xFF */
  {
    snprintf((char*)tx_buffer, sizeof(tx_buffer),
      "WHO_AM_I not programmed, WHO_AM_I = 0x%X\r\n", who);
    tx_com(tx_buffer, strlen((char const*)tx_buffer));
  }
  else
  {
    snprintf((char*)tx_buffer, sizeof(tx_buffer), /* Custom WHO_AM_I by customer */
      "Device initialized, check custom WHO_AM_I = 0x%X\r\n", who);
    tx_com(tx_buffer, strlen((char const*)tx_buffer));
  }

  /* Set auto-mode */
  nmh1000_auto_mode_set(&dev_ctx, AUTO_ON);

  /* Set polarity */
  nmh1000_polarity_set(&dev_ctx, NMH_SETB); //OUT pin asserts when magnet is present

  /* Set ODR */
  nmh1000_user_odr_set(&dev_ctx, NMH1000_USER_ODR_10X_HSP); //100Hz
  nmh1000_user_odr_get(&dev_ctx, &val);

  nmh1000_out_flag = false;

  return 0;

}


/**
  * @brief  The application entry point.
  * @retval int
  */
int32_t nmh1000_magnetic_field_detect(void)
{
  nmh1000_mag_t nmh;
  nmh1000_status_t status;

  /* Infinite loop */
  while (1)
  {
    /* In ISR Mode we do not need to check Data Ready Register.
     * The receipt of interrupt will indicate data is ready. */
    if (!nmh1000_out_flag)
    { /* Loop, if new sample is not available. */
      continue;
    }
    else
    { /*! Clear the data ready flag, it will be set again by the ISR. */
      nmh1000_out_flag = false;
    }

    /* Read NMH1000 Magnetic Field Value */
    nmh1000_status_get(&dev_ctx, &status);

    /* Read NMH1000 Magnetic Field Value */
    nmh1000_magnetic_field_get(&dev_ctx, &nmh);

    /* Convert magnetic field raw value into Gauss units */
    nmh.mag_field = nmh1000_convert_to_G(nmh.mag_data);

    if (count)
    {
      /* Print Magnetic Strength field to the serial terminal. */
      snprintf((char*)tx_buffer, sizeof(tx_buffer),
          "Event: %d, Magnetic Field: %0.1fG Status: 0x%2x\r\n",
           count, nmh.mag_field, (uint8_t)status.w);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));
    }


    /* The MDR and MDO bits are set by the sensor and must be
     * explicitly cleared by the host to ensure correct DRDY
     * signal operation. */
    nmh1000_status_mdr_clear(&dev_ctx);
    nmh1000_status_mdo_clear(&dev_ctx);


#if defined(NUCLEO_F401RE)
    /* Green LED Blinks on magnetic presence, once per occurrence. */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

    platform_delay(200);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
#endif

    count++;

  }

  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle  customizable argument. In this examples is used in
 *           order to select the correct sensor bus handler.
 * @param  reg     register to write
 * @param  bufp    pointer to data to write in register reg
 * @param  len     number of consecutive register to write
 *
 */
int32_t platform_write(void* handle, uint8_t reg,
  const uint8_t* bufp, uint16_t len)
{
#if defined(NUCLEO_F401RE)
  if (HAL_I2C_Mem_Write(handle, NMH1000_I2C_ADD_VAL, reg,
    I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000) != HAL_OK)
  {
    return -1;
  }
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle, NMH1000_I2C_ADD_VAL & 0xFE, reg, (uint8_t*)bufp, len);
#endif

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle  customizable argument. In this examples is used in
 *           order to select the correct sensor bus handler.
 * @param  reg     register to read
 * @param  bufp    pointer to buffer that store the data read
 * @param  len     number of consecutive register to read
 *
 */
static int32_t platform_read(void* handle, uint8_t reg,
  uint8_t* bufp, uint16_t len)
{
#if defined(NUCLEO_F401RE)
  if (HAL_I2C_Mem_Read(handle, NMH1000_I2C_ADD_VAL, reg,
    I2C_MEMADD_SIZE_8BIT, bufp, len, 1000) != HAL_OK)
  {
    return -1;
  }
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, NMH1000_I2C_ADD_VAL & 0xFE, reg, bufp, len);
#endif

  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer   buffer to transmit
 * @param  len       number of byte to send
 *
 */
static void tx_com(uint8_t* tx_buffer, uint16_t len)
{
#if defined(NUCLEO_F401RE)
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
#elif defined(SPC584B_DIS)
  sd_lld_write(&SD2, tx_buffer, len);
#endif
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms    delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
#if defined(NUCLEO_F401RE)
  HAL_Delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

