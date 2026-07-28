/*
 ******************************************************************************
 * @file    fxls8964af_sdcd_tap_detect.c
 * @author  Sensors Software Solution Team
 * @brief   This file show an example for FXLS8964AF using SDCD function
 *          to configure sensor for detecting tap or transient event.
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
 * - NUCLEO_F401RE + FRDM-STBA-A8964
 * - STEVAL_MKI109D + FXLS8964AF (DIL-24)
 * - DISCOVERY_SPC584B + FRDM-STBA-A8964
 *
 * Used interfaces:
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default)
 *
 * STEVAL_MKI109D     - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default)
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

//#define STEVAL_MKI109D   /* little endian */
//#define NUCLEO_F401RE    /* little endian */
//#define SPC584B_DIS      /* big endian */


/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109D)
/* MKI109D: Define communication interface */
#define SENSOR_BUS hspi1

/* MKI109D: Vdd and Vddio power supply values */
#define FXLS8964AF_VDD 3.3f
#define FXLS8964AF_VDDIO 3.3f

#elif defined(NUCLEO_F401RE)
/* NUCLEO_F401RE: Define communication interface */
#define SENSOR_BUS hi2c1

#elif defined(SPC584B_DIS)
/* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>

#if defined(NUCLEO_F401RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#elif defined(STEVAL_MKI109D)
#include "board.h"
#include "usbd_cdc_if.h"

#elif defined(SPC584B_DIS)
#include "components.h"

#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fxls8964af_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BOOT_TIME         10
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t tx_buffer[1000];
stmdev_ctx_t dev_ctx;
fxls8964af_fs_t fs;
fxls8964af_odr_t odr;
fxls8964af_pin_int_route_t int_pin;
fxls8964af_aslp_config_t aslp_config;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t gFxls8964af_sdcd_ot_flag = 0;
/* USER CODE END 0 */
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void *handle);

/* FXLS8964AF SDCD WT ISR */
void fxls8964af_sdcd_ot_irq_handler(void)
{
    /* Set flag to indicate Sensor has raised SDCD WT interrupt */
	gFxls8964af_sdcd_ot_flag = 1;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Initialize FXLS8964AF Device */
int32_t fxls8964af_init_example(void)
{
  uint8_t who;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
  platform_init(dev_ctx.handle);

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Reset device */
  fxls8964af_reset(&dev_ctx);

  fxls8964af_device_id_get(&dev_ctx, &who);
  if (FXLS8964_WHO_AM_I_VAL == who)
  {
    snprintf((char *)tx_buffer, sizeof(tx_buffer),
            " Successfully Initialized Gemini with WHO_AM_I = 0x%X\r\n",who);
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }
  else
  {
    snprintf((char *)tx_buffer, sizeof(tx_buffer),
            "Bad WHO_AM_I = 0x%X\r\n",who);
    tx_com(tx_buffer, strlen((char const *)tx_buffer));
    return -1;
  }

  /* Set full scale range */
  fxls8964af_full_scale_set(&dev_ctx, FXLS8964AF_4g);
  fxls8964af_full_scale_get(&dev_ctx, &fs);

  /* Set ODR */
  fxls8964af_wake_odr_set(&dev_ctx, FXLS8964AF_ODR_100HZ);
  fxls8964af_sleep_odr_set(&dev_ctx, FXLS8964AF_ODR_25HZ);
  fxls8964af_wake_odr_get(&dev_ctx, &odr);

  /* Set SDCD for Tap/Transient Detection Event */
  fxls8964af_sdcd_config_set(&dev_ctx, FXLS8964AF_TAP_CONFIG);

  /* Set the ASLP count to 5sec */
  aslp_config.aslp_cnt_lsb = 0xF4;
  aslp_config.aslp_cnt_msb = 0x01;
  fxls8964af_aslp_config_set(&dev_ctx, aslp_config);

  /* Enable SDCD OT interrupt and route to INT1 pin */
  fxls8964af_sdcd_ot_int_enable(&dev_ctx);
  fxls8964af_pin_int1_route_set(&dev_ctx, FXLS8964AF_SDCD_OT_INT_PIN);

  return 0;

}

/**
  * @brief  The application entry point.
  * @retval int
  */
int32_t fxls8964af_sdcd_tap_detect(void)
{

  uint8_t skip = 0;


  snprintf((char *)tx_buffer, sizeof(tx_buffer),
      "\r\n\r\n Perform Tap or Transient motion and sensor will detect the event \r\n");
  tx_com(tx_buffer, strlen((char const *)tx_buffer));


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    if (1 == gFxls8964af_sdcd_ot_flag)
    {
      gFxls8964af_sdcd_ot_flag = 0;
      skip++;

      if (skip > 1)
      {
        /* Wake Mode Detected. */
        snprintf((char *)tx_buffer, sizeof(tx_buffer),
            "\r\n\r\n Tap Event Detected on INT1....Count = %d\r\n", (skip - 1));
        tx_com(tx_buffer, strlen((char const *)tx_buffer));

        snprintf((char *)tx_buffer, sizeof(tx_buffer),
            "\r\n Waiting for another Tap or Transient event to occur \r\n");
        tx_com(tx_buffer, strlen((char const *)tx_buffer));
      }
    }
    /* USER CODE END WHILE */
  }
  /* USER CODE END 3 */
  return 0;
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
int32_t platform_write(void *handle, uint8_t reg,
                       const uint8_t *bufp, uint16_t len)
{
#if defined(NUCLEO_F401RE)
  if (HAL_I2C_Mem_Write(handle, FXLS8964AF_I2C_ADD_SA0_0, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000) != HAL_OK)
  {
    return -1;
  }
#elif defined(STEVAL_MKI109D)
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY); //8CLKs dummy
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  FXLS8964AF_I2C_ADD_SA0_0 & 0xFE, reg, (uint8_t*) bufp, len);
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
static int32_t platform_read(void *handle, uint8_t reg,
                      uint8_t *bufp, uint16_t len)
{
#if defined(NUCLEO_F401RE)
  if (HAL_I2C_Mem_Read(handle, FXLS8964AF_I2C_ADD_SA0_0, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000) != HAL_OK)
  {
    return -1;
  }
#elif defined(STEVAL_MKI109D)
  reg |= 0x80;
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY); //8CLKs dummy
  HAL_SPI_Receive(handle, bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, FXLS8964AF_I2C_ADD_SA0_0 & 0xFE, reg, bufp, len);
#endif

  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
#if defined(NUCLEO_F401RE)
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
#elif defined(STEVAL_MKI109D)
  CDC_Transmit_FS(tx_buffer, len);
#elif defined(SPC584B_DIS)
  sd_lld_write(&SD2, tx_buffer, len);
#endif
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
#if defined(NUCLEO_F401RE) || defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(STEVAL_MKI109D)
  delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void *handle)
{
#if defined(STEVAL_MKI109D)
  struct spi_conf spi_conf;

  /* init SPI bus communication */
  spi_conf.wire = WIRE_4_EXT;
  spi_init(&spi_conf);

  /* set VDD/VDDIO on DIL24 */
  set_vdd(FXLS8964AF_VDD);
  set_vddio(FXLS8964AF_VDDIO);
  delay(100);

#endif
}
/* USER CODE END 0 */
