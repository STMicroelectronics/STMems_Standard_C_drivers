/*
 ******************************************************************************
 * @file    fxps7550_analog_output_stream.c
 * @author  Sensors Software Solution Team
 * @brief   This example demonstrates how to interface the FXPS7550A4S pressure
 *          sensor using its analog output with an embedded MCU ADC subsystem.
 *
 *          The FXPS75504AS provides a ratio-metric analog voltage output that
 *          is proportional to the applied pressure over a calibrated operating
 *          range spanning from 20 kPa to 250 kPa. The sensor analog output
 *          pin is connected directly to an MCU ADC IN0.
 *
 *          Continuously reads FXPS75504AS analog output using ADC interrupt mode.
 *          ADC must be configured in continuous mode to generate samples at a 
 *          constant rate. It applies 16× oversampling (averaging) to improve 
 *          measurement precision. Converts samples to pressure (kPa) 
 *          and prints results at ~5 Hz.
 */
/*
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
 * - NUCLEO_F401RE + FRDM7X-INTERFACE + BRKTSTBAPA7550A
 *
 * Used interfaces:
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - Sensor side: Analog only output
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please un-comment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

#define NUCLEO_F401RE    /* little endian */


/* ADC handler by platform */
#if defined(NUCLEO_F401RE)
/* NUCLEO_F401RE: Define ADC interface */
#define ADC_INTF hadc1
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "fxps7550_analog.h"

#if defined(NUCLEO_F401RE)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"

#endif

#define REGION_REPORT_PRINT 1U /* Comment to skip printing region report */
#define PRINT(...)                                                      \
do {                                                                    \
    snprintf((char *)tx_buffer, sizeof(tx_buffer), __VA_ARGS__);        \
    tx_com(tx_buffer, strlen((char const *)tx_buffer));                 \
} while (0)

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define VCC_SUPPLY       3.3f
#define BOOT_TIME         10
/* USER CODE END PM */

/* USER CODE BEGIN PV */
static uint8_t tx_buffer[1000];
stmdev_ctx_t dev_ctx;
bool adc_conversion_done = false;
uint32_t adc_value = 0;
typedef enum {
	DEMO_LOOP_START      = 0,
	DEMO_GET_SAMPLE      = 1,
	DEMO_IDLE            = 2
} demo_seq_t;
/* USER CODE END PV */

demo_seq_t seq;
uint8_t sample_count;
uint32_t adcVal_avg;
uint32_t adcAcc =0;
uint32_t last_print;


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
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);


/* FXPS7550 Sensor ISR Routine */
void fxps7550_adc_irq_handler(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    /* Set flag to indicate a new ADC conversion is done
     * and ready to be read */
    adc_conversion_done = true;

  }
}

/* Main Example --------------------------------------------------------------*/
int32_t fxps7550_analog_example(void)
{
  int32_t ret = FXPS7550_SUCCESS;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &ADC_INTF;

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  
  seq = DEMO_LOOP_START;

  char pn_str[12];
  fxps7550_device_id_get(&dev_ctx, pn_str);
  PRINT("Sensor PN: %s\r\n", pn_str);

  /* Start ADC conversion in interrupt mode.
   * ADC is configured for continuous conversion mode.
   * End-of-conversion event selected as end of sequence (ADC_EOC_SEQ_CONV).
   */
  ret = HAL_ADC_Start_IT(&hadc1);

  return ret;
}


/* Read FXPS7550 Analog Output Stream */
int32_t fxps7550_analog_stream(void)
{
  float pressure_kPa;
  uint16_t pPressure;
  int32_t ret = 0;

  switch (seq) {
  case DEMO_LOOP_START:

	  last_print = HAL_GetTick();
	  seq = DEMO_GET_SAMPLE;
	  break;

  case DEMO_GET_SAMPLE:

	  if (adc_conversion_done)
	  {
		fxps7550_pressure_raw_get(&dev_ctx, &pPressure);
		adcAcc += pPressure;
	  	sample_count++;

	  	if (sample_count == 16)  // 16× oversampling
	  	{
	  	  adcVal_avg = adcAcc / sample_count;

	  	  /* Transfer function for FXPS7550A4S Pressure Sensor */
		  pressure_kPa = fxps7550_pressure_raw_to_kpa(adcVal_avg);

	  	  adcAcc = 0;
	  	  sample_count = 0;
		  adc_conversion_done = false;
	  	}
	  }
	  break;

  default:

	  seq = DEMO_GET_SAMPLE;
      break;
  }


  /* Print pressure data 5 times per second */
  if ((HAL_GetTick() - last_print) > 200)
  {
      PRINT("Pressure: %.3f kPa\r\n", pressure_kPa);
      last_print = HAL_GetTick();
  }

  return ret;
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
#endif
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
  int32_t ret = FXPS7550_SUCCESS;

#if defined(NUCLEO_F401RE)
  ret = FXPS7550_ERROR_READ_ONLY;
#endif

  return ret;
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
  int32_t ret = FXPS7550_SUCCESS;
  uint16_t buf_temp;

#if defined(NUCLEO_F401RE)
  buf_temp = HAL_ADC_GetValue(handle);

  bufp[1] = (buf_temp >> 8) & 0xFF;
  bufp[0] = buf_temp & 0xFF;
#endif

  return ret;
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
#if defined(NUCLEO_F401RE)
  HAL_Delay(ms);
#endif
}

