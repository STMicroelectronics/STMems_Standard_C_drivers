/*
 ******************************************************************************
 * @file    fxps7550_read_data_polling.c
 * @author  Sensors Software Solution Team
 * @brief   This example demonstrates how to configure the FXPS7550D4S sensor
 *          in polling mode to poll data ready event and read sensor samples
 *          on data ready event.
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
 * - NUCLEO_F401RE + FRDM7X-INTERFACE + BRKFXPS7XXX-PCB
 * - STEVAL_MKI109D + FRDM7X-INTERFACE + BRKFXPS7XXX-PCB
 * - DISCOVERY_SPC584B + FRDM7X-INTERFACE + BRKFXPS7XXX-PCB
 *
 * Used interfaces:
 *
 * NUCLEO_STM32F401RE - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI
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

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

//#define STEVAL_MKI109D     /* little endian */
//#define NUCLEO_F401RE      /* little endian */
//#define NUCLEO_F401RE_SPI  /* little endian */
//#define SPC584B_DIS        /* big endian */


/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109D) || defined(NUCLEO_F401RE_SPI)
/* MKI109D: Define communication interface */
#define SENSOR_BUS hspi1

/* MKI109D: Vdd and Vddio power supply values */
#define FXPS7550_VDD 3.3f
#define FXPS7550_VDDIO 3.3f

#elif defined(NUCLEO_F401RE)
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
#include "fxps7550_reg.h"

#if defined(NUCLEO_F401RE) || defined(NUCLEO_F401RE_SPI)
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"

#elif defined(STEVAL_MKI109D)
#include "board.h"
#include "usbd_cdc_if.h"

#elif defined(SPC584B_DIS)
#include "components.h"

#endif

#define REGION_REPORT_PRINT 1U /* Comment to skip printing region report */
#define PRINT(...)                                                      \
do {                                                                    \
    snprintf((char *)tx_buffer, sizeof(tx_buffer), __VA_ARGS__);        \
    tx_com(tx_buffer, strlen((char const *)tx_buffer));                 \
} while (0)


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */
#define BOOT_TIME         10

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t tx_buffer[1000];
stmdev_ctx_t dev_ctx;
/* USER CODE END PV */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
fxps7550_crc_param_t pCrcSettings;
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
void spi_crc_settings_init(void);

int32_t fxps7550_spi_frame_pack(void *handle, eStandardSpiCommand* aeCommands,
        uint8_t* address,uint8_t* data,uint8_t len, uint8_t* spiFrame);

int32_t fxps7550_spi_frame_verify(void *handle, eStandardSpiCommand* aeCommands, uint8_t* address,
        uint8_t len, uint8_t* responses, uint16_t *data, tStandardSpiTransactionStatus* tStatus);

void regionc_report_print(fxps7550_regionc_t *regc)
{
  PRINT("\r\nFXPS7550 Region C Report: \r\n");
  PRINT("-----------------------------------\r\n");
  PRINT(" ICTYPEID: 0x%X\r\n", regc->ic_type_id);
  PRINT(" ICREVID: 0x%X\r\n", regc->ic_rev_id);
  PRINT(" ICMFGID: 0x%X\r\n", regc->ic_mfg_id);
  PRINT(" PN0: 0x%X\r\n", regc->PN0);
  PRINT(" PN1: 0x%X\r\n", regc->PN1);
  PRINT(" SN0: 0x%X%X%X%X%X\r\n", regc->dev_sn[0], regc->dev_sn[1], regc->dev_sn[2],
          regc->dev_sn[3], regc->dev_sn[4]);
  PRINT(" ASICWFR: 0x%X\r\n", regc->asic_wfr);
  PRINT(" ASICWFR_X: 0x%X\r\n", regc->asic_wfr_x);
  PRINT(" ASICWFR_Y: 0x%X\r\n", regc->asic_wfr_y);
}

void regiond_report_print(fxps7550_regiond_t *regd)
{
  PRINT("\r\nFXPS7550 Region D Report: \r\n");
  PRINT("-----------------------------------\r\n");
  PRINT(" ASICWLOT_L: 0x%X\r\n", regd->wlot[0]);
  PRINT(" ASICWLOT_H: 0x%X\r\n", regd->wlot[1]);
  PRINT(" ASICWLOT: 0x%X\r\n", regd->asic_wlot);
}

/* Main Example --------------------------------------------------------------*/
/* Initialize FXPS7550 Device */
int32_t fxps7550_init_example(void)
{

 // fxps7550_user_odr_val_t val;
  int32_t ret = FXPS7550_SUCCESS;
  fxps7550_devstat_t devstat;
  fxps7550_dsp_stat_t dspstat;
  uint8_t who;

  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Init test platform */
  platform_init(dev_ctx.handle);
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Initialize crc settings */
  spi_crc_settings_init();

  /* FXPS7550 Reset */
  fxps7550_reset(&dev_ctx);
  platform_delay(1); //Allow at least 1ms for the part to reset

  /* Dummy read to flush the data in communication buffer */
  fxps7550_devstat_get(&dev_ctx, &devstat);

  fxps7550_device_id_get(&dev_ctx, &who);
  if (FXPS7550_WHOAMI_VAL == who)
  {
    PRINT("Successfully Initialized DBAP Sensor with WHO_AM_I = 0x%X\r\n",who);
  }
  else
  {
    PRINT("Device not found, check WHO_AM_I = 0x%X\r\n",who);
  }

  //read the sensor information
  fxps7550_regionc_t regionc;
  fxps7550_regiond_t regiond;

  /* get fxps7550 offset and sensitivity based on table */
  fxpsdev.pressureOffset = part_tbl.pabOffSet;
  fxpsdev.pressureSens = part_tbl.pabSense;

  /* Load Region C allows user to read trace-ability registers located
   * in the address range: 0xC0 to 0xCF */
  ret = fxps7550_region_c_get(&dev_ctx, &regionc);

  /* Load Region D allows user to read trace-ability registers located
   * in the address range: 0xD0 to 0xDF */
  ret = fxps7550_load_region(&dev_ctx, CONFIG_D_REGISTERS);
  if(ret != FXPS7550_SUCCESS){
    return ret;
  }
  ret = fxps7550_region_d_get(&dev_ctx, &regiond);

#if defined(REGION_REPORT_PRINT)
  regionc_report_print(&regionc);
  regiond_report_print(&regiond);
#endif

  /* Run Self-Test */
  PRINT("\r\nExecuting self-test\r\n");

  ret = fxps7550_startup_self_test(&dev_ctx, FXPS7550_FIXED_ST, FXPS7550_ST_2, 1);
  PRINT("Fixed ST: %s",
        (ret == FXPS7550_SUCCESS) ? "ST fixed test Passed\r\n" : "ST fixed test failed\r\n");

  ret = fxps7550_startup_self_test(&dev_ctx, FXPS7550_DIGITAL_ST, FXPS7550_ST_2, 100);
  PRINT("Digital ST: %s",
        (ret == FXPS7550_SUCCESS) ? "ST digital test Passed\r\n" : "ST digital test failed\r\n");

  /* Get DSP STatus after self-test run. */
  fxps7550_dsp_stat_get(&dev_ctx, &dspstat);
  PRINT("dsp status --> ST_ACTIVE: %u,\r\n"
        "               ST_ERROR: %u,\r\n"
        "               ST_INCOMPLETE: %u\r\n",
        dspstat.b.st_active, dspstat.b.st_error, dspstat.b.st_incmplt);
  fxps7550_devstat_get(&dev_ctx, &devstat);
  PRINT("dsp status --> DEV_STAT: %b,\r\n",devstat.w);


  //Set LPF and DSP- Datatype settings
  ret = fxps7550_lpf_set(&dev_ctx, LPFTYPE_1000HZ_4POLE);
  ret = fxps7550_datatype0_set(&dev_ctx, DATATYPE_PRESSURE_ABS);
  ret = fxps7550_datatype1_set(&dev_ctx, DATATYPE_TEMPERATURE);


  return ret;
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int32_t fxps7550_read_data_polling(void)
{

  fxps7550_devstat_t devstat;
  fxps7550_devstat1_t devstat1;
  fxps7550_devstat2_t devstat2;
  fxps7550_devstat3_t devstat3;
  uint16_t pressure_data = 0;
  float fpressure;
  uint8_t utemp = 0;

  PRINT("\r\nPressure and Temperature readings\r\n");
  /* Infinite loop */
  while (1)
  {

    /* USER CODE BEGIN WHILE */
    /* Wait for data ready from the FXPS7550. */
    fxps7550_devstat_get(&dev_ctx, &devstat);
    fxps7550_devstat1_get(&dev_ctx, &devstat1);
    fxps7550_devstat2_get(&dev_ctx, &devstat2);
    fxps7550_devstat3_get(&dev_ctx, &devstat3);

    fxps7550_get_pressure_raw(&dev_ctx, &pressure_data);
    fxps7550_get_temperature_raw(&dev_ctx, &utemp);

    fpressure = fxps7550_pressure_raw_to_kpa(pressure_data);
    utemp = fxps7550_temp_raw_to_degc(utemp);

    PRINT(" Pressure = %0.3f kPa, Temperature = %d degC\r\n",
            (float)fpressure, utemp);

    // Print around 10 pressure samples/s
    platform_delay(100);

  }
  
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
  int32_t ret = FXPS7550_SUCCESS;

#if defined(NUCLEO_F401RE)
  if (HAL_I2C_Mem_Write(handle, FXPS7550_I2C_ADDR_VAL, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000) != HAL_OK)
  {
    ret = -1;
  }
#elif defined(STEVAL_MKI109D) || defined(NUCLEO_F401RE_SPI)

  eStandardSpiCommand eCommand = eStandardSpiCommand_WriteReg;
  uint16_t dataBuffer = 0;
  uint8_t spiFrame[4];
  uint8_t addresses[2] = { (uint8_t)reg,(uint8_t)reg };
  tStandardSpiTransactionStatus tStatus ;
  uint8_t buffpp[4];

  if(len > 2){
    return FXPS7550_INVALIDPARAM_ERR;
  }
  if(NULL == handle)
  {
      return FXPS7550_BAD_HANDLE;
  }

  fxps7550_spi_frame_pack(handle, &eCommand, addresses, (uint8_t *)bufp, 1, spiFrame);

  /* Host command – SPI transaction. */
  /* The first transaction initiates communication with the sensor.
   * The response from this transaction is ignored.
   */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, spiFrame, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  /* The second transaction is performed to retrieve valid response
   * data from the sensor.
   * Refer to the Standard 32-bit SPI protocol [DS].
   */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Receive(handle, buffpp, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  fxps7550_spi_frame_verify(handle, &eCommand, addresses, len, buffpp, &dataBuffer, &tStatus);

#elif defined(SPC584B_DIS)
  i2c_lld_write(handle, FXPS7550_I2C_ADD_VAL & 0xFE, reg, (uint8_t*) bufp, len);
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

#if defined(NUCLEO_F401RE)
  if (HAL_I2C_Mem_Read(handle, FXPS7550_I2C_ADDR_VAL, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000) != HAL_OK)
  {
    ret = -1;
  }
#elif defined(STEVAL_MKI109D) || defined(NUCLEO_F401RE_SPI)

  eStandardSpiCommand eCommand[2] = {eStandardSpiCommand_ReadReg,eStandardSpiCommand_ReadReg};
  uint8_t dummy[2] = {0,0} ;
  uint16_t dataBuffer = 0;
  uint8_t spiFrame[4];
  uint8_t addresses[2] = { (uint8_t)reg,(uint8_t)reg };
  tStandardSpiTransactionStatus tStatus ;
  uint16_t data = 0;
  uint8_t buffpp[4];

  if(len > 2)
  {
    return FXPS7550_INVALIDPARAM_ERR;
  }

  if((NULL == handle) || (NULL == bufp))
  {
    return false;
  }

  fxps7550_spi_frame_pack(handle, eCommand, addresses, dummy,2, spiFrame);

  /* Host command – SPI transaction. */
  /* The first transaction initiates communication with the sensor.
   * The response from this transaction is ignored.
   */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, spiFrame, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  /* The second transaction is performed to retrieve valid response
   * data from the sensor.
   * Refer to the Standard 32-bit SPI protocol [DS].
   */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Receive(handle, buffpp, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  fxps7550_spi_frame_verify(handle, eCommand, addresses, len, buffpp, &dataBuffer, &tStatus);

  if(tStatus.bCrcError ){
    return FXPS7550_READ_ERR;
  }
  data = dataBuffer;
  if(len == 1)
  {
    *bufp = ((uint8_t)reg) % 2 == 1 ? (uint8_t)(data >> 8) : (uint8_t)data;
  }else
  {
    *(uint16_t*)bufp = dataBuffer;
  }


#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, FXPS7550_I2C_ADD_VAL & 0xFE, reg, bufp, len);
#endif

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
  set_vdd(FXPS7550_VDD);
  set_vddio(FXPS7550_VDDIO);
  delay(100);

#endif
}

int32_t fxps7550_spi_frame_pack(void *handle, eStandardSpiCommand* aeCommands,
        uint8_t* address,uint8_t* data,uint8_t len, uint8_t* spiFrame)
{
  uint32_t tempData = 0;
  int32_t ret = 0;

  if((NULL == handle) || (NULL == aeCommands))
  {
    return -1;
  }

  tempData|=( (aeCommands[0]<<28) & 0xF0000000);
  tempData|=( (address[0]<<16) & 0x00FF0000);
  tempData|=( (data[0]<<8) & 0x0000FF00);
  tempData|=( (fxps7550_calculate_crc(&pCrcSettings,tempData)) & 0x000000FF);

  spiFrame[0] = (tempData>>24) & 0xFF;
  spiFrame[1] = (tempData>>16) & 0xFF;
  spiFrame[2] = (tempData>> 8) & 0xFF;
  spiFrame[3] = (tempData    ) & 0xFF;

  return ret;
}

int32_t fxps7550_spi_frame_verify(void *handle, eStandardSpiCommand* aeCommands, uint8_t* address,
        uint8_t len, uint8_t* responses, uint16_t *data, tStandardSpiTransactionStatus* tStatus)
{
  int32_t ret = 0;
  uint32_t response;
  uint8_t command;

  response = ((responses[0]<<24) &0xFF000000) |
             ((responses[1]<<16) &0xFF0000) |
             ((responses[2]<< 8) &0xFF00) |
             ((responses[3]<< 0) &0xFF);

  if(tStatus!=NULL)
  {
    tStatus->bCrcError = !fxps7550_crc_check(response&((1<<pCrcSettings.crcLength)-1), &pCrcSettings,response);
    command = ((response>>28)&0xF);
    command = (command << 1) | ((command>> 3) &0x01);

    tStatus->eBasicStatus = (response>>26)&0x03;
    tStatus->eDetailedStatus = eStandardSpiDetailedStatus_None;
  }

  if(*aeCommands == eStandardSpiCommand_ReadReg || *aeCommands == eStandardSpiCommand_WriteReg)
  {
    *data = (response>>8)&0xFFFF;
  }
  else
  {
    *data = (response>>10)&0xFFFF;
    if(tStatus!=NULL)
      tStatus->eDetailedStatus = (response>>8)&0x3;
  }

  return ret;
}


void spi_crc_settings_init(void){
  pCrcSettings.eType = SEEDTYPE_NON_DIRECT;
  pCrcSettings.polynomial = CRC_POLY;
  pCrcSettings.initialValue = CRC_SEED;
  pCrcSettings.crcLength = CRC_LENGTH;
  pCrcSettings.dataBitStart = 0;
  pCrcSettings.dataLength = 32;

}
