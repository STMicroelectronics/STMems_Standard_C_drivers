/*
 ******************************************************************************
 * @file    nbp8fd4s_read_pressure_data_polling.c
 * @author  Sensors Software Solution Team
 * @brief   This example demonstrates how to configure the NBP8FD4S sensor.
 *          It also illustrates the sequence the host must follow to request
 *          data from the sensor and gain access to all device registers.
 *          During each iteration, the host requests pressure,
 *          temperature, and voltage measurements.
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
 * - NUCLEO_F401RE + NBP8FD4S
 * - STEVAL_MKI109D + NBP8FD4S (DIL-24)
 * - DISCOVERY_SPC584B + NBP8FD4S
 *
 * Used interfaces:
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - Sensor side: SPI(Default)
 *
 * STEVAL_MKI109D     - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: SPI(Default)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition */
/*
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

//#define STEVAL_MKI109V3
//#define NUCLEO_F401RE
#define NUCLEO_F401RE_SPI    /* little endian */
//#define SPC584B_DIS


/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109D)
/* MKI109D: Define communication interface */
#define SENSOR_BUS hspi1

/* MKI109D: Vdd and Vddio power supply values */
#define NBP8FD4S_VDD 3.3f
#define NBP8FD4S_VDDIO 3.3f

#elif defined(NUCLEO_F401RE)
/* NUCLEO_F401RE: Define communication interface */
#define SENSOR_BUS hi2c1
#elif defined(NUCLEO_F401RE_SPI)
#define SENSOR_BUS hspi1 /* Customization for SPI on NUCLEO-F401RE */

#elif defined(SPC584B_DIS)
 /* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS I2CD1

#endif

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "nbp8fd4s_reg.h"
#include "spi_communication.h"

#if defined(NUCLEO_F401RE) || defined(NUCLEO_F401RE_SPI)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"
#include "iwdg.h"

#elif defined(STEVAL_MKI109D)
#include "board.h"
#include "usbd_cdc_if.h"

#elif defined(SPC584B_DIS)
#include "components.h"

#endif

#define BOOT_TIME 10

#define REGION_REPORT_PRINT 1U /* Comment to skip printing region report */
#define PRINT(...)                                                      \
do {                                                                    \
    snprintf((char *)tx_buffer, sizeof(tx_buffer), __VA_ARGS__);        \
    tx_com(tx_buffer, strlen((char const *)tx_buffer));                 \
} while (0)

/* Private variables ---------------------------------------------------------*/
static uint8_t tx_buffer[1000];
nbp8fd4s_ctx_t dev_ctx;

volatile uint8_t gNbp8fd4s_int_flag = 0;

/*
 *   WARNING:
 *   Functions declared in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_read_write(void *handle, uint8_t* TxBuf, uint8_t *Rxbuf, uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void *handle);

static void vfnWaitForIntRdyAsserted (uint8_t u8Pol);
static void vfnWaitForIntRdyIdle (uint8_t u8Pol);
static bool bIsIntRdyAsserted (uint8_t u8Pol);
static void vfnEnableRdyIntInterrupt (uint8_t u8Pol);
static void vfnCfgWakeupPinLow (void);

/********************************************************
 * Private functions
 *******************************************************/
static void handle_interrupt_example (void);
static void read_sensor_data_polling_example (void);
static void set_polling_delay (uint32_t* delay);
static int32_t i32RequestCommunication (void);


/********************************************************
 * Code
 *******************************************************/

/* NBP8FD4S Data Ready ISR */
void nbp8fd4s_drdy_irq_handler(void)
{
  /* Set flag to indicate Sensor has raised Data Ready interrupt */
	gNbp8fd4s_int_flag = 1;
}

/* Initialize NBP8FD4S Device */
int32_t nbp8fd4s_init_example(void)
{
	uint8_t u8fw_version;
	uint8_t u8fw_derivative;
	int32_t ret;

    dev_ctx.write_reg = platform_read_write;
    dev_ctx.read_reg = platform_read_write;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &SENSOR_BUS;

    /* Init test platform */
    platform_init(dev_ctx.handle);
    /* Wait sensor boot time */
    platform_delay(BOOT_TIME);

    /* NBP Initialization */
    if (0 == i32RequestCommunication())
    {
    	ret = nbp8fd4s_firmware_info_get(&dev_ctx, &u8fw_version, &u8fw_derivative);

    	/* As an example, configure the fixed threshold to 536 counts = 150 kPa */
    	ret |= nbp8fd4s_fixed_threshold_set(&dev_ctx, 536);

    	/* Add additional read/write commands here as needed */

	    /* Always release communication at the end of the SPI sequence */
	    nbp8fd4s_release_communication(&dev_ctx);

    	if (0 == ret) // print data if no issue during reading
    	{
    		PRINT("\r\nInitialization successful.\r\n");
    		PRINT("NBP firmware version: %#x \t NBP firmware derivative:%#x \r\n\r\n", u8fw_version, u8fw_derivative);
    	}
    	else
    	{
    		PRINT("Initialization not successful, problem during register read/write.");
    	}
    }
    else
    {
		PRINT("Problem during initialization, communication could not be established.");
    }

    /* Clear interrupt flag */
    gNbp8fd4s_int_flag = 0;

    /* Enable interrupt on INT/READY pin - Default polarity is configured in the NBP (assert at high level) */
    vfnEnableRdyIntInterrupt(INTPOL_ASSERT_HIGH);

    return 0;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int32_t nbp8fd4s_read_pressure_data_polling(void)
{
	uint32_t nbp8fd4s_polling_delay;

	/* Set polling delay depending on NBP sampling period configured */
	set_polling_delay(&nbp8fd4s_polling_delay);

	/* Wait for first sensor data to be available */
	HAL_Delay(nbp8fd4s_polling_delay);


  /* Infinite loop */
  while (1)
  {

	/* Interrupts and pressure change algorithms are enabled by default: check if NBP triggered an interrupt.
	 * To disable all interrupts, disable the pressure change algorithms and clear FVERR, STERR, SENSERR and
	 * SENSRDY bits in INTTRIG register */
	if (gNbp8fd4s_int_flag)
	{
		gNbp8fd4s_int_flag = 0;

		/* Example to handle an interrupt notification */
		handle_interrupt_example();
	}

    /* Trigger communication to read sensor data */
	read_sensor_data_polling_example();

	/* Wait for NBP to take new measurements before reading again */
    HAL_Delay(nbp8fd4s_polling_delay);

  }

  return 0;
}

/*********************************************************************************
 * Private functions
 *********************************************************************************/

/*
 * @brief  Example of function handling an interrupt notification from the NBP
 *
 */
static void handle_interrupt_example (void)
{
	int32_t ret = -1;
	uint8_t u8Status_reg;
	uint8_t u8Senstatus_reg;

	/* Wait for INT pulse to complete before starting SPI transfers */
	vfnWaitForIntRdyIdle(INTPOL_ASSERT_HIGH);

	if (0 == nbp8fd4s_check_communication_ready(&dev_ctx))
	{
		/* Clear the interrupt on NBP side - if not done, NBP will trigger the same notification again */
		nbp8fd4s_clear_INTF_flag(&dev_ctx);
		/* Read STATUS register to know the origin of the interrupt */
		ret = nbp8fd4s_read_reg(&dev_ctx, NBP8FD4S_STATUS, &u8Status_reg);
		ret |= nbp8fd4s_read_reg(&dev_ctx, NBP8FD4S_SENSTATUS, &u8Senstatus_reg);

		/* Insert additional register read/write here as desired */

		/* Always release communication at the end of the SPI sequence */
		nbp8fd4s_release_communication(&dev_ctx);
	}


	/* If no error during communication, display data to the user */
	if (0 == ret)
	{
		PRINT("\r\nInterrupt occurred, STATUS = %#x \t SENSTATUS = %#x \r\n \r\n", u8Status_reg, u8Senstatus_reg);
	}

	return;
}

/*
 * @brief  Example of function triggering communication to read sensor data
 *
 */
static void read_sensor_data_polling_example (void)
{
	int32_t ret = -1;
	float pressure;
	float temperature;
	float voltage;
	uint8_t u8Status_reg;

	/* First, request communication with the NBP */
	if (0 == i32RequestCommunication())
	{
		ret = nbp8fd4s_pressure_get(&dev_ctx, &pressure);
		ret |= nbp8fd4s_temperature_get(&dev_ctx, &temperature);
		ret |= nbp8fd4s_voltage_get(&dev_ctx, &voltage);
		ret |= nbp8fd4s_read_reg(&dev_ctx, NBP8FD4S_STATUS, &u8Status_reg);

		/* Insert additional register read/write here as desired */

		/* Always release communication at the end of the SPI sequence */
		nbp8fd4s_release_communication(&dev_ctx);
	}


	/* If no error during communication, display data to the user */
	if (0 == ret)
	{
		PRINT("Pressure: %.2f kPa \t Temperature: %.2f C \t Voltage: %.2f V \t STATUS: %#x \r\n", pressure, temperature, voltage, u8Status_reg);
	}
}

/*
 * @brief  Sets the polling delay according to the NBP sampling period configured
 *
 * @param  delay	pointer to delay value, in ms (ptr)
 */
static void set_polling_delay (uint32_t* delay)
{
	uint8_t psp;
	int32_t ret = -1;

	*delay = 500; // default to 500ms

	/* Read sampling period configured in NBP */
	if (0 == i32RequestCommunication())
	{
		ret = nbp8fd4s_read_reg(&dev_ctx, NBP8FD4S_PSP, &psp);

		/* Always release communication at the end of the SPI sequence */
		nbp8fd4s_release_communication(&dev_ctx);
	}

	/* Set polling delay to twice the sampling period
	 * If polling delay is shorter than the sampling period, the NBP will not
	 * have time to take new data */
	if (0 == ret)
	{
		if (NBP8FD4S_PSP_10_MS == psp)
			*delay = 20;
		else if (NBP8FD4S_PSP_20_MS == psp)
			*delay = 40;
		else if (NBP8FD4S_PSP_40_MS == psp)
			*delay = 80;
		else if (NBP8FD4S_PSP_70_MS == psp)
			*delay = 140;
		else if (NBP8FD4S_PSP_135_MS == psp)
			*delay = 270;
		else if (NBP8FD4S_PSP_510_MS == psp)
			*delay = 1020;
		else if (NBP8FD4S_PSP_1000_MS == psp)
			*delay = 2000;
	}

	return;
}

/*
 * @brief  Requests communication with the NBP
 *
 * @retval  0 if communication could be established, non-0 otherwise
 */
static int32_t i32RequestCommunication (void)
{
	int32_t i32status = -1;

	/* Check that the NBP is not requesting communication first */
	if (false == bIsIntRdyAsserted(INTPOL_ASSERT_HIGH))
	{
		/* Lower CS_B pin and wait for acknowledgment */
		vfnCfgWakeupPinLow();
		vfnWaitForIntRdyAsserted(INTPOL_ASSERT_HIGH);
		/* Perform dummy read to clear error on NBP side */
		nbp8fd4s_dummy_read(&dev_ctx);
		/* Check NBP is ready for communication */
		i32status = nbp8fd4s_check_communication_ready(&dev_ctx);

		/* The assertion of READY pin triggered an interrupt: clear the flag before going back */
		gNbp8fd4s_int_flag = 0;
	}

	return i32status;
}

/*
 * @brief  Waits for the INT/RDY pin to be asserted
 *
 * @param  u8Pol    polarity of the RDY/INT pin
 */
static void vfnWaitForIntRdyAsserted (uint8_t u8Pol)
{
	/* WARNING: there is no timeout implemented. A final application should
	 * implement a timeout or enable the watchdog.
	 */

	if (INTPOL_ASSERT_HIGH == u8Pol) // Asserted at '1', idle at '0'
	{
		while (HAL_GPIO_ReadPin(M_INT3_GPIO_Port, M_INT3_Pin) == GPIO_PIN_RESET);
	}
	else // Asserted at '0', idle at '1'
	{
		while (HAL_GPIO_ReadPin(M_INT3_GPIO_Port, M_INT3_Pin) == GPIO_PIN_SET);
	}
}

/*
 * @brief  Waits for the INT/RDY pin to be idle
 *
 * @param  u8Pol    polarity of the RDY/INT pin
 */
static void vfnWaitForIntRdyIdle (uint8_t u8Pol)
{
	/* WARNING: there is no timeout implemented. A final application should
	 * implement a timeout or enable the watchdog.
	 */
	if (INTPOL_ASSERT_HIGH == u8Pol) // Asserted at '1', idle at '0'
	{
		while (HAL_GPIO_ReadPin(M_INT3_GPIO_Port, M_INT3_Pin) == GPIO_PIN_SET);
	}
	else // Asserted at '0', idle at '1'
	{
		while (HAL_GPIO_ReadPin(M_INT3_GPIO_Port, M_INT3_Pin) == GPIO_PIN_RESET);
	}
}

/*
 * @brief  Indicates if the INT/RDY pin is idle or asserted
 *
 * @param  u8Pol    polarity of the RDY/INT pin
 * @retval       TRUE if pin asserted, FALSE otherwise
 */
static bool bIsIntRdyAsserted (uint8_t u8Pol)
{
	bool pin_asserted;

	if (INTPOL_ASSERT_HIGH == u8Pol) // Asserted at '1', idle at '0'
	{
		if (HAL_GPIO_ReadPin(M_INT3_GPIO_Port, M_INT3_Pin) == 1)
			pin_asserted = 1;
		else
			pin_asserted = 0;
	}
	else // Asserted at '0', idle at '1'
	{
		if (HAL_GPIO_ReadPin(M_INT3_GPIO_Port, M_INT3_Pin) == 0)
			pin_asserted = 1;
		else
			pin_asserted = 0;
	}

	return pin_asserted;
}

/*
 * @brief  Enable the interrupt on the MCU pin connected to RDY/INT pin
 *
 * @param  u8Pol    polarity of the RDY/INT pin
 */
static void vfnEnableRdyIntInterrupt (uint8_t u8Pol)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (INTPOL_ASSERT_HIGH == u8Pol)
	{
		/* M_INT3 Interruption asserted at '1', idle at '0' */
		GPIO_InitStruct.Pin  = M_INT3_Pin;
   	    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
   		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else // Asserted at '0', idle at '1'
	{
		/* M_INT3 Interruption asserted at '0', idle at '1' */
		GPIO_InitStruct.Pin  = M_INT3_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	}

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/*
 * @brief  Configure the WAKEUP/CS pin as output low
 *         to trigger a transfer request on the NBP8 side
 *
 */
static void vfnCfgWakeupPinLow (void)
{
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
}


/*
 * @brief  Read write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  TxBuf     transmission buffer
 * @param  Rxbuf     reception buffer
 * @param  len       number of bytes to read/write
 * @retval       Interface status (MANDATORY: return 0 -> no Error)
 */
static int32_t platform_read_write(void *handle, uint8_t* TxBuf, uint8_t* Rxbuf, uint16_t len)
{

#if defined(NUCLEO_F401RE_SPI)

	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(handle, TxBuf, Rxbuf, len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);

#endif

  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 * @retval       Interface status (MANDATORY: return 0 -> no Error)
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
#if defined(NUCLEO_F401RE) || defined(NUCLEO_F401RE_SPI)
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
#if defined(NUCLEO_F401RE) || defined(NUCLEO_F401RE_SPI)
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
  set_vdd(NBP8FD4S_VDD);
  set_vddio(NBP8FD4S_VDDIO);
  delay(100);
#endif
}

