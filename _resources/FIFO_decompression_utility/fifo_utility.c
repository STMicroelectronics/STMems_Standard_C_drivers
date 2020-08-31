/*
 ******************************************************************************
 * @file    fifo_utility.c
 * @author  Sensor Solutions Software Team
 * @brief   Utility for managing data compression in smart FIFO.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "fifo_utility.h"

/**
  * @defgroup  FIFO utility
  * @brief     This file provides a set of functions needed to  manage data
  *            compression in smart FIFO.
  * @{
  *
  */

/* Private constants  --------------------------------------------------------*/
#define ODR_XL_MASK              (0x0FU)
#define ODR_XL_SHIFT             (0x00U)
#define BDR_XL_MASK              (0x0FU)
#define BDR_XL_SHIFT             (0x00U)

#define ODR_GY_MASK              (0xF0U)
#define ODR_GY_SHIFT             (0x04U)
#define BDR_GY_MASK              (0xF0U)
#define BDR_GY_SHIFT             (0x04U)

#define BDR_VSENS_MASK           (0x0FU)
#define BDR_VSENS_SHIFT          (0x00U)

#define TAG_COUNTER_MASK         (0x06U)
#define TAG_SENSOR_MASK          (0xF8U)
#define TAG_COUNTER_SHIFT        (0x01U)
#define TAG_SENSOR_SHIFT         (0x03U)

#define TAG_GY                   (0x01U)
#define TAG_XL                   (0x02U)
#define TAG_TEMP                 (0x03U)
#define TAG_TS                   (0x04U)
#define TAG_ODRCHG               (0x05U)
#define TAG_XL_UNCOMPRESSED_T_2  (0x06U)
#define TAG_XL_UNCOMPRESSED_T_1  (0x07U)
#define TAG_XL_COMPRESSED_2X     (0x08U)
#define TAG_XL_COMPRESSED_3X     (0x09U)
#define TAG_GY_UNCOMPRESSED_T_2  (0x0AU)
#define TAG_GY_UNCOMPRESSED_T_1  (0x0BU)
#define TAG_GY_COMPRESSED_2X     (0x0CU)
#define TAG_GY_COMPRESSED_3X     (0x0DU)
#define TAG_EXT_SENS_0           (0x0EU)
#define TAG_EXT_SENS_1           (0x0FU)
#define TAG_EXT_SENS_2           (0x10U)
#define TAG_EXT_SENS_3           (0x11U)
#define TAG_STEP_COUNTER         (0x12U)
#define TAG_GAME_RV              (0x13U)
#define TAG_GEOM_RV              (0x14U)
#define TAG_NORM_RV              (0x15U)
#define TAG_GYRO_BIAS            (0x16U)
#define TAG_GRAVITIY             (0x17U)
#define TAG_MAG_CAL              (0x18U)
#define TAG_EXT_SENS_NACK        (0x19U)

#define TAG_VALID_LIMIT          (0x19U)

#define TIMESTAMP_FREQ          (40000U)

/* Private typedef -----------------------------------------------------------*/
typedef enum {
  ST_FIFO_COMPRESSION_NC,
  ST_FIFO_COMPRESSION_NC_T_1,
  ST_FIFO_COMPRESSION_NC_T_2,
  ST_FIFO_COMPRESSION_2X,
  ST_FIFO_COMPRESSION_3X
} st_fifo_compression_type;

/* Private functions ---------------------------------------------------------*/
/*  Functions declare in this section are defined at the end of this file.    */
static uint8_t has_even_parity(uint8_t x);
static st_fifo_sensor_type get_sensor_type(uint8_t tag);
static st_fifo_compression_type get_compression_type(uint8_t tag);
static uint8_t is_tag_valid(uint8_t tag);
static void get_diff_2x(int16_t diff[6], uint8_t input[6]);
static void get_diff_3x(int16_t diff[9], uint8_t input[6]);
static void byte_cpy(uint8_t *destination, uint8_t *source, uint32_t len);

/* Private variables ---------------------------------------------------------*/
static uint8_t tag_counter_old     = 0x00U;
static float_t bdr_xl              = 0.0f;
static float_t bdr_gy              = 0.0f;
static float_t bdr_vsens           = 0.0f;
static float_t bdr_xl_old          = 0.0f;
static float_t bdr_gy_old          = 0.0f;
static float_t bdr_max             = 0.0f;
static uint32_t timestamp          = 0;
static uint32_t last_timestamp_xl  = 0;
static uint32_t last_timestamp_gy  = 0;
static uint8_t bdr_chg_xl_flag     = 0;
static uint8_t bdr_chg_gy_flag     = 0;
static int16_t last_data_xl[3]     = {0};
static int16_t last_data_gy[3]     = {0};

/**
  * @defgroup  FIFO_pubblic_functions
  * @brief     This section provide a set of usefull APIs for managing data
  *            compression in smart FIFO.
  * @{
  *
  */

/**
  * @brief  Initialize the FIFO utility library.
  *
  * @param  bdr_xl_in         batch data rate for accelerometer sensor in Hz,
  *                           pass 0 Hz if odrchg_en is set to 1 or timestamp
  *                           is stored in FIFO.
  * @param  bdr_gy_in         batch data rate for gyro sensor in Hz,
  *                           pass 0 Hz if odrchg_en is set to 1 or timestamp
  *                           is stored in FIFO.
  * @param  bdr_vsens_in      batch data rate for virtual sensor in Hz,
  *                           pass 0 Hz if odrchg_en is set to 1 or timestamp
  *                           is stored in FIFO.
  *
  * @retval st_fifo_status    ST_FIFO_OK /  ST_FIFO_ERR
  *
  */
st_fifo_status st_fifo_init(float_t    bdr_xl_in,
                            float_t    bdr_gy_in,
                            float_t    bdr_vsens_in)
{
  uint32_t i;
  st_fifo_status ret = ST_FIFO_ERR;

  if ((bdr_xl_in < 0.0f) || (bdr_gy_in < 0.0f) || (bdr_vsens_in < 0.0f)) {
    ret = ST_FIFO_ERR;
  }
  else {

    tag_counter_old = 0x00U;
    bdr_xl = bdr_xl_in;
    bdr_gy = bdr_gy_in;
    bdr_vsens = bdr_vsens_in;
    bdr_xl_old = bdr_xl_in;
    bdr_gy_old = bdr_gy_in;
    bdr_max = ( ( bdr_xl  > bdr_gy )    ? bdr_xl  : bdr_gy );
    bdr_max = ( ( bdr_max > bdr_vsens ) ? bdr_max : bdr_vsens );
    timestamp = 0;
    bdr_chg_xl_flag = 0;
    bdr_chg_gy_flag = 0;
    last_timestamp_xl = 0;
    last_timestamp_gy = 0;

    for (i = 0; i < 3U; i++) {
      last_data_xl[i] = 0;
      last_data_gy[i] = 0;

      ret = ST_FIFO_OK;
    }
  }

  return ret;
}

/**
  * @brief  Decompress a compressed raw FIFO stream.
  *
  * @param  fifo_out_slot     decoded output stream.(ptr)
  * @param  fifo_raw_slot     compressed raw input data stream.(ptr)
  * @param  out_slot_size     decoded stream size.(ptr)
  * @param  stream_size       raw input stream size.
  *
  * @retval st_fifo_status    ST_FIFO_OK /  ST_FIFO_ERR
  *
  */
st_fifo_status st_fifo_decompress(st_fifo_out_slot *fifo_out_slot,
                                  st_fifo_raw_slot *fifo_raw_slot,
                                  uint16_t *out_slot_size,
                                  uint16_t stream_size)
{
  uint16_t j = 0;
  int16_t data[3];
  uint8_t tag;
  uint8_t tag_counter;
  uint8_t diff_tag_counter;
  uint8_t bdr_acc_cfg;
  uint8_t bdr_gyr_cfg;
  uint8_t bdr_vsens_cfg;
  uint32_t last_timestamp;
  int16_t diff[9];

  float_t bdr_acc_vect[]  = {    0,   13    ,  26,   52,  104,
                               208,  416    , 833, 1666, 3333,
                              6666,    1.625,   0,    0,    0,
                                 0 };

  float_t bdr_gyr_vect[] = {   0,   13,   26,   52, 104, 208, 416,
                             833, 1666, 3333, 6666,   0,   0,   0,
                               0,    0};

  float_t bdr_vsens_vect[] = { 0, 13, 26, 52, 104    , 208, 416,
                               0,  0,  0,  0,   1.625,   0,   0,
                               0,  0};

  for (uint16_t i = 0; i < stream_size; i++) {

    tag = (fifo_raw_slot[i].fifo_data_out[0] & TAG_SENSOR_MASK);
    tag = tag >> TAG_SENSOR_SHIFT;

    tag_counter = (fifo_raw_slot[i].fifo_data_out[0] & TAG_COUNTER_MASK);
    tag_counter = tag_counter >> TAG_COUNTER_SHIFT;

    if ((has_even_parity(fifo_raw_slot[i].fifo_data_out[0]) == 0U) ||
        (is_tag_valid(tag) == 0U)){
      return ST_FIFO_ERR;
    }

    if ((tag_counter != (tag_counter_old)) && (bdr_max != 0.0f)) {

      if (tag_counter < tag_counter_old){
        diff_tag_counter = tag_counter + 4U - tag_counter_old;
      }
      else{
        diff_tag_counter = tag_counter - tag_counter_old;
      }

      timestamp += (TIMESTAMP_FREQ / (uint32_t)bdr_max) * diff_tag_counter;
    }

    if (tag == TAG_ODRCHG) {

      bdr_acc_cfg = (fifo_raw_slot[i].fifo_data_out[6] & BDR_XL_MASK);
      bdr_acc_cfg = bdr_acc_cfg >> BDR_XL_SHIFT;

      bdr_gyr_cfg = (fifo_raw_slot[i].fifo_data_out[6] & BDR_GY_MASK);
      bdr_gyr_cfg = bdr_gyr_cfg >> BDR_GY_SHIFT;

      bdr_vsens_cfg =(fifo_raw_slot[i].fifo_data_out[3] & BDR_VSENS_MASK);
      bdr_vsens_cfg = bdr_vsens_cfg >> BDR_VSENS_SHIFT;

      bdr_xl_old = bdr_xl;
      bdr_gy_old = bdr_gy;

      bdr_xl = bdr_acc_vect[bdr_acc_cfg];
      bdr_gy = bdr_gyr_vect[bdr_gyr_cfg];
      bdr_vsens = bdr_vsens_vect[bdr_vsens_cfg];
      bdr_max = ((bdr_xl > bdr_gy) ? bdr_xl : bdr_gy);
      bdr_max = ((bdr_max > bdr_vsens) ? bdr_max : bdr_vsens);

      bdr_chg_xl_flag = 1;
      bdr_chg_gy_flag = 1;

      } else if (tag == TAG_TS) {

        byte_cpy( (uint8_t*)&timestamp, &fifo_raw_slot[i].fifo_data_out[1], 4);

      } else {

      st_fifo_compression_type compression_type = get_compression_type(tag);
      st_fifo_sensor_type sensor_type = get_sensor_type(tag);

      switch (compression_type){
        case ST_FIFO_COMPRESSION_NC:
          if (tag == TAG_STEP_COUNTER){
            byte_cpy((uint8_t*)&fifo_out_slot[j].timestamp,
                     &fifo_raw_slot[i].fifo_data_out[3], 4);
          }
          else{
            fifo_out_slot[j].timestamp = timestamp;
          }

          fifo_out_slot[j].sensor_tag = sensor_type;
          byte_cpy(fifo_out_slot[j].raw_data,
                   &fifo_raw_slot[i].fifo_data_out[1], 6);

          if (sensor_type == ST_FIFO_ACCELEROMETER) {
            byte_cpy((uint8_t*)last_data_xl, fifo_out_slot[j].raw_data, 6);
            last_timestamp_xl = timestamp;
            bdr_chg_xl_flag = 0;
          }

          if (sensor_type == ST_FIFO_GYROSCOPE) {
            byte_cpy((uint8_t*)last_data_gy, fifo_out_slot[j].raw_data, 6);
            last_timestamp_gy = timestamp;
            bdr_chg_gy_flag = 0;
          }

          j++;
          break;
        case ST_FIFO_COMPRESSION_NC_T_1:
          fifo_out_slot[j].sensor_tag = get_sensor_type(tag);
          byte_cpy(fifo_out_slot[j].raw_data,
                   &fifo_raw_slot[i].fifo_data_out[1], 6);

          if (sensor_type == ST_FIFO_ACCELEROMETER) {


            if (bdr_chg_xl_flag != 0U){
              last_timestamp = (last_timestamp_xl +
                                (TIMESTAMP_FREQ / (uint32_t)bdr_xl_old));
            }
            else{
              last_timestamp = ((uint32_t)timestamp -
                                ((uint32_t)TIMESTAMP_FREQ / (uint32_t)bdr_xl));
            }

            fifo_out_slot[j].timestamp = last_timestamp;
            byte_cpy((uint8_t*)last_data_xl,
                     (uint8_t*) fifo_out_slot[j].raw_data, 6);
            last_timestamp_xl = last_timestamp;
          }

          if (sensor_type == ST_FIFO_GYROSCOPE) {


            if (bdr_chg_gy_flag != 0U){
              last_timestamp = (last_timestamp_gy +
                                (TIMESTAMP_FREQ / (uint32_t)bdr_gy_old));
            }
            else{
              last_timestamp = (timestamp -
                                (TIMESTAMP_FREQ / (uint32_t)bdr_gy));
            }

            fifo_out_slot[j].timestamp = last_timestamp;
            byte_cpy((uint8_t*)last_data_gy, fifo_out_slot[j].raw_data, 6);
            last_timestamp_gy = last_timestamp;
          }

          j++;
          break;
        case ST_FIFO_COMPRESSION_NC_T_2:
          fifo_out_slot[j].sensor_tag = get_sensor_type(tag);
          byte_cpy(fifo_out_slot[j].raw_data,
                   &fifo_raw_slot[i].fifo_data_out[1], 6);

          if (sensor_type == ST_FIFO_ACCELEROMETER) {
            if (bdr_chg_xl_flag != 0U){
              last_timestamp = (last_timestamp_xl +
                                (TIMESTAMP_FREQ / (uint32_t)bdr_xl_old));
            }
            else{
              last_timestamp = (timestamp -
                                ((2U * TIMESTAMP_FREQ) / (uint32_t) bdr_xl));
            }

            fifo_out_slot[j].timestamp = last_timestamp;
            byte_cpy((uint8_t*)last_data_xl, fifo_out_slot[j].raw_data, 6);
            last_timestamp_xl = last_timestamp;
          }
          if (sensor_type == ST_FIFO_GYROSCOPE) {

            if (bdr_chg_gy_flag != 0U){
              last_timestamp = (last_timestamp_gy +
                                (TIMESTAMP_FREQ / (uint32_t)bdr_gy_old));
            }
            else{
              last_timestamp = (timestamp -
                                (2U * TIMESTAMP_FREQ / (uint32_t)bdr_gy));
            }

            fifo_out_slot[j].timestamp = last_timestamp;
            byte_cpy((uint8_t*)last_data_gy,
                     (uint8_t*)fifo_out_slot[j].raw_data, 6);
            last_timestamp_gy = last_timestamp;
          }

          j++;
          break;
        case ST_FIFO_COMPRESSION_2X:
          get_diff_2x(diff, &fifo_raw_slot[i].fifo_data_out[1]);

          fifo_out_slot[j].sensor_tag = sensor_type;

          if (sensor_type == ST_FIFO_ACCELEROMETER) {
            data[0] = last_data_xl[0] + diff[0];
            data[1] = last_data_xl[1] + diff[1];
            data[2] = last_data_xl[2] + diff[2];
            byte_cpy(fifo_out_slot[j].raw_data,(uint8_t*)data, 6);
            fifo_out_slot[j].timestamp =
                        (timestamp - (2U * TIMESTAMP_FREQ / (uint32_t)bdr_xl));

            byte_cpy((uint8_t*)last_data_xl, fifo_out_slot[j].raw_data, 6);
          }

          if (sensor_type == ST_FIFO_GYROSCOPE) {
            data[0] = last_data_gy[0] + diff[0];
            data[1] = last_data_gy[1] + diff[1];
            data[2] = last_data_gy[2] + diff[2];
            byte_cpy(fifo_out_slot[j].raw_data,(uint8_t*)data, 6);
            fifo_out_slot[j].timestamp =
                        (timestamp - (2U * TIMESTAMP_FREQ / (uint32_t)bdr_gy));

            byte_cpy((uint8_t*)last_data_gy, fifo_out_slot[j].raw_data, 6);
          }

          j++;

          fifo_out_slot[j].sensor_tag = sensor_type;

          if (sensor_type == ST_FIFO_ACCELEROMETER) {
            last_timestamp = (timestamp - (TIMESTAMP_FREQ / (uint32_t)bdr_xl));
            data[0] = last_data_xl[0] + diff[3];
            data[1] = last_data_xl[1] + diff[4];
            data[2] = last_data_xl[2] + diff[5];
            byte_cpy(fifo_out_slot[j].raw_data,(uint8_t*)data, 6);
            fifo_out_slot[j].timestamp = last_timestamp;
            byte_cpy((uint8_t*)last_data_xl, fifo_out_slot[j].raw_data, 6);
            last_timestamp_xl = last_timestamp;
          }

          if (sensor_type == ST_FIFO_GYROSCOPE) {
            last_timestamp = (timestamp - (TIMESTAMP_FREQ / (uint32_t)bdr_gy));
            data[0] = last_data_gy[0] + diff[3];
            data[1] = last_data_gy[1] + diff[4];
            data[2] = last_data_gy[2] + diff[5];
            byte_cpy(fifo_out_slot[j].raw_data,(uint8_t*)data, 6);
            fifo_out_slot[j].timestamp = last_timestamp;
            byte_cpy((uint8_t*)last_data_gy, fifo_out_slot[j].raw_data, 6);
            last_timestamp_gy = last_timestamp;
          }

          j++;
          break;
        default: //(compression_type == ST_FIFO_COMPRESSION_3X)

          get_diff_3x(diff, &fifo_raw_slot[i].fifo_data_out[1]);

          fifo_out_slot[j].sensor_tag = sensor_type;

          if (sensor_type == ST_FIFO_ACCELEROMETER) {
            data[0] = last_data_xl[0] + diff[0];
            data[1] = last_data_xl[1] + diff[1];
            data[2] = last_data_xl[2] + diff[2];
            byte_cpy(fifo_out_slot[j].raw_data,(uint8_t*)data, 6);
            fifo_out_slot[j].timestamp =
                        (timestamp - (2U * TIMESTAMP_FREQ / (uint32_t)bdr_xl));
            byte_cpy((uint8_t*)last_data_xl, fifo_out_slot[j].raw_data, 6);
          }

          if (sensor_type == ST_FIFO_GYROSCOPE) {
            data[0] = last_data_gy[0] + diff[0];
            data[1] = last_data_gy[1] + diff[1];
            data[2] = last_data_gy[2] + diff[2];
            byte_cpy(fifo_out_slot[j].raw_data,(uint8_t*)data, 6);
            fifo_out_slot[j].timestamp =
                        (timestamp - (2U * TIMESTAMP_FREQ / (uint32_t)bdr_gy));
            byte_cpy((uint8_t*)last_data_gy,
                     (uint8_t*)fifo_out_slot[j].raw_data, 6);
          }

          j++;

          fifo_out_slot[j].sensor_tag = sensor_type;

          if (sensor_type == ST_FIFO_ACCELEROMETER) {
            data[0] = last_data_xl[0] + diff[3];
            data[1] = last_data_xl[1] + diff[4];
            data[2] = last_data_xl[2] + diff[5];
            byte_cpy(fifo_out_slot[j].raw_data,(uint8_t*)data, 6);
            fifo_out_slot[j].timestamp =
                             (timestamp -(TIMESTAMP_FREQ / (uint32_t)bdr_xl));
            byte_cpy((uint8_t*)last_data_xl, fifo_out_slot[j].raw_data, 6);
          }

          if (sensor_type == ST_FIFO_GYROSCOPE) {
            data[0] = last_data_gy[0] + diff[3];
            data[1] = last_data_gy[1] + diff[4];
            data[2] = last_data_gy[2] + diff[5];
            byte_cpy(fifo_out_slot[j].raw_data,(uint8_t*)data, 6);
            fifo_out_slot[j].timestamp =
                             (timestamp - (TIMESTAMP_FREQ / (uint32_t)bdr_gy));
            byte_cpy((uint8_t*)last_data_gy, fifo_out_slot[j].raw_data, 6);
          }

          j++;

          fifo_out_slot[j].timestamp = timestamp;
          fifo_out_slot[j].sensor_tag = sensor_type;

          if (sensor_type == ST_FIFO_ACCELEROMETER) {
            data[0] = last_data_xl[0] + diff[6];
            data[1] = last_data_xl[1] + diff[7];
            data[2] = last_data_xl[2] + diff[8];
            byte_cpy(fifo_out_slot[j].raw_data,(uint8_t*)data, 6);
            byte_cpy((uint8_t*)last_data_xl, fifo_out_slot[j].raw_data, 6);
            last_timestamp_xl = timestamp;
          }

          if (sensor_type == ST_FIFO_GYROSCOPE) {
            data[0] = last_data_gy[0] + diff[6];
            data[1] = last_data_gy[1] + diff[7];
            data[2] = last_data_gy[2] + diff[8];
            byte_cpy(fifo_out_slot[j].raw_data,(uint8_t*)data, 6);
            byte_cpy((uint8_t*)last_data_gy, fifo_out_slot[j].raw_data, 6);
            last_timestamp_gy = timestamp;
          }

          j++;
          break;
        }

        *out_slot_size = j;
      }

    tag_counter_old = tag_counter;
  }

  return ST_FIFO_OK;
}

/**
  * @brief  Sort FIFO stream from older to newer timestamp.
  *
  * @param  fifo_out_slot     decoded output stream.(ptr)
  * @param  out_slot_size     decoded srteam size.
  *
  */
void st_fifo_sort(st_fifo_out_slot *fifo_out_slot, uint16_t out_slot_size)
{

  int32_t j, i;
  st_fifo_out_slot temp;

  for (i = 1; i < (int32_t)out_slot_size; i++) {

    byte_cpy((uint8_t*)&temp, (uint8_t*)&fifo_out_slot[i],
             sizeof(st_fifo_out_slot));

    j = i - 1;

    while ((j >= 0) && (fifo_out_slot[j].timestamp > temp.timestamp)) {
      byte_cpy((uint8_t*)&fifo_out_slot[j + 1], (uint8_t*)&fifo_out_slot[j],
               sizeof(st_fifo_out_slot));
      j--;
    }

    byte_cpy((uint8_t*)&fifo_out_slot[j + 1], (uint8_t*)&temp,
             sizeof(st_fifo_out_slot));
  }

}

/**
  * @brief  Return the number of a sensor tag occurrency in a
  *         decoded FIFO stream.
  *
  * @param  fifo_out_slot     decoded output stream.(ptr)
  * @param  out_slot_size     decoded srteam size.
  * @param  sensor_type       The name of the sensor that is need
  *                           to count occurrences.
  *
  * @retval uint16_t          the number of a sensor tag occurrency in a
  *                           decoded FIFO stream.
  *
  */
uint16_t  st_fifo_get_sensor_occurrence(st_fifo_out_slot *fifo_out_slot,
                                        uint16_t out_slot_size,
                                        st_fifo_sensor_type sensor_type)
{
  uint16_t occurrence = 0;

  for (uint16_t i = 0; i < out_slot_size; i++) {

    if (fifo_out_slot[i].sensor_tag == sensor_type){
      occurrence++;
    }

  }

  return occurrence;
}

/**
  * @brief  This function extracts all the data of a specific sensor
  *         from a decoded FIFO stream.
  *
  * @param  sensor_out_slot   data of a specific sensor.(ptr)
  * @param  fifo_out_slot     decoded output stream.(ptr)
  * @param  out_slot_size     decoded srteam size.
  * @param  sensor_type       The name of the sensor that is need
  *                           to extract data.
  *
  */
void st_fifo_extract_sensor(st_fifo_out_slot *sensor_out_slot,
                            st_fifo_out_slot *fifo_out_slot,
                            uint16_t  out_slot_size,
                            st_fifo_sensor_type sensor_type)
{
  uint16_t temp_i = 0;

  for (uint16_t i = 0; i < out_slot_size; i++) {

    if (fifo_out_slot[i].sensor_tag == sensor_type) {

      byte_cpy((uint8_t*)&sensor_out_slot[temp_i], (uint8_t*)&fifo_out_slot[i],
               sizeof(st_fifo_out_slot));

      temp_i++;
    }

  }
}

/**
  * @}
  *
  */

/**
  * @defgroup  FIFO private functions
  * @brief     This section provide a set of private low-level functions
  *            used by pubblic APIs.
  * @{
  *
  */

/**
  * @brief  This function indicate if a raw tag is valid or not.
  *
  * @param  tag               tag to be analyzed.
  *
  * @retval uint8_t           valid(1) / invalid(0) tag.
  *
  */
static uint8_t is_tag_valid(uint8_t tag)
{
  uint8_t ret;

  if (tag > TAG_VALID_LIMIT){
    ret = 0;
  }
  else{
    ret = 1;
  }

  return ret;
}

/**
  * @brief  This function convert a raw tag in a sensor type
  *
  * @param  tag                    tag to be analyzed.
  *
  * @retval st_fifo_sensor_type    Sensor type.
  *
  */
static st_fifo_sensor_type get_sensor_type(uint8_t tag)
{
  st_fifo_sensor_type ret;
  switch (tag) {
    case TAG_GY:
      ret = ST_FIFO_GYROSCOPE;
      break;
    case TAG_XL:
      ret =  ST_FIFO_ACCELEROMETER;
      break;
    case TAG_TEMP:
      ret =  ST_FIFO_TEMPERATURE;
      break;
    case TAG_EXT_SENS_0:
      ret =  ST_FIFO_EXT_SENSOR0;
      break;
    case TAG_EXT_SENS_1:
      ret =  ST_FIFO_EXT_SENSOR1;
      break;
    case TAG_EXT_SENS_2:
      ret =  ST_FIFO_EXT_SENSOR2;
      break;
    case TAG_EXT_SENS_3:
      ret =  ST_FIFO_EXT_SENSOR3;
      break;
    case TAG_STEP_COUNTER:
      ret =  ST_FIFO_STEP_COUNTER;
      break;
    case TAG_XL_UNCOMPRESSED_T_2:
      ret =  ST_FIFO_ACCELEROMETER;
      break;
    case TAG_XL_UNCOMPRESSED_T_1:
      ret =  ST_FIFO_ACCELEROMETER;
      break;
    case TAG_XL_COMPRESSED_2X:
      ret =  ST_FIFO_ACCELEROMETER;
      break;
    case TAG_XL_COMPRESSED_3X:
      ret =  ST_FIFO_ACCELEROMETER;
      break;
    case TAG_GY_UNCOMPRESSED_T_2:
      ret =  ST_FIFO_GYROSCOPE;
      break;
    case TAG_GY_UNCOMPRESSED_T_1:
      ret =  ST_FIFO_GYROSCOPE;
      break;
    case TAG_GY_COMPRESSED_2X:
      ret =  ST_FIFO_GYROSCOPE;
      break;
    case TAG_GY_COMPRESSED_3X:
      ret =  ST_FIFO_GYROSCOPE;
      break;
    case TAG_GAME_RV:
      ret =  ST_FIFO_6X_GAME_RV;
      break;
    case TAG_GEOM_RV:
      ret =  ST_FIFO_6X_GEOM_RV;
      break;
    case TAG_NORM_RV:
      ret =  ST_FIFO_9X_RV;
      break;
    case TAG_GYRO_BIAS:
      ret =  ST_FIFO_GYRO_BIAS;
      break;
    case TAG_GRAVITIY:
      ret =  ST_FIFO_GRAVITY;
      break;
    case TAG_MAG_CAL:
      ret =  ST_FIFO_MAGNETOMETER_CALIB;
      break;
    case TAG_EXT_SENS_NACK:
      ret =  ST_FIFO_EXT_SENSOR_NACK;
      break;
    default:
      ret =  ST_FIFO_NONE;
      break;

  }
  return ret;
}

/**
  * @brief  This function convert a raw tag in a type of compression
  *
  * @param  tag                         tag to be analyzed.
  *
  * @retval st_fifo_compression_type    Compression type.
  *
  */
static st_fifo_compression_type get_compression_type(uint8_t tag)
{
  st_fifo_compression_type ret;
  switch (tag) {
    case TAG_GY:
      ret =  ST_FIFO_COMPRESSION_NC;
      break;
    case TAG_XL:
      ret =  ST_FIFO_COMPRESSION_NC;
      break;
    case TAG_TEMP:
      ret =  ST_FIFO_COMPRESSION_NC;
      break;
    case TAG_EXT_SENS_0:
      ret =  ST_FIFO_COMPRESSION_NC;
      break;
    case TAG_EXT_SENS_1:
      ret =  ST_FIFO_COMPRESSION_NC;
      break;
    case TAG_EXT_SENS_2:
      ret =  ST_FIFO_COMPRESSION_NC;
      break;
    case TAG_EXT_SENS_3:
      ret =  ST_FIFO_COMPRESSION_NC;
      break;
    case TAG_STEP_COUNTER:
      ret =  ST_FIFO_COMPRESSION_NC;
      break;
    case TAG_XL_UNCOMPRESSED_T_2:
      ret =  ST_FIFO_COMPRESSION_NC_T_2;
      break;
    case TAG_XL_UNCOMPRESSED_T_1:
      ret =  ST_FIFO_COMPRESSION_NC_T_1;
      break;
    case TAG_XL_COMPRESSED_2X:
      ret =  ST_FIFO_COMPRESSION_2X;
      break;
    case TAG_XL_COMPRESSED_3X:
      ret =  ST_FIFO_COMPRESSION_3X;
      break;
    case TAG_GY_UNCOMPRESSED_T_2:
      ret =  ST_FIFO_COMPRESSION_NC_T_2;
      break;
    case TAG_GY_UNCOMPRESSED_T_1:
      ret =  ST_FIFO_COMPRESSION_NC_T_1;
      break;
    case TAG_GY_COMPRESSED_2X:
      ret =  ST_FIFO_COMPRESSION_2X;
      break;
    case TAG_GY_COMPRESSED_3X:
      ret =  ST_FIFO_COMPRESSION_3X;
      break;
    default:
      ret =  ST_FIFO_COMPRESSION_NC;
      break;
  }
  return ret;
}

/**
  * @brief  This function check the parity of a byte.
  *
  * @param  x                 Byte to be analyzed.
  *
  * @retval uint8_t           Sensor type defined in st_fifo_compression_type.
  *
  */
static uint8_t has_even_parity(uint8_t x)
{
  uint8_t count = 0x00, i, b = 0x01;
  uint8_t ret = 1;

  for (i = 0U; i < 8U; i++) {
    if( ( x & (b << i) ) != 0x00U){
      count++;
    }
  }

  if ((count & 0x01U) == 0x01U) {
    ret = 0;
  }

  return ret;
}

/**
  * @brief  Convert raw data FIFO into compressed data (2x).
  *
  * @param  diff[6]           Compressed data (2x).
  * @param  input[6]          FIFO raw word without tag.
  *
  */
static void get_diff_2x(int16_t diff[6], uint8_t input[6])
{
  uint8_t i;
  for (i = 0; i < 6U; i++){
    if (input[i] < 128U ){
      diff[i] = (int16_t)input[i];
    }
    else {
      diff[i] = ((int16_t)input[i] - 256);
    }
  }
}

/**
  * @brief  Convert raw data FIFO into compressed data (3x).
  *
  * @param  diff[6]           Compressed data (3x).
  * @param  input[6]          fifo raw word without tag.
  *
  */
static void get_diff_3x(int16_t diff[9], uint8_t input[6])
{
  uint16_t decode_temp;
  uint16_t dummy;

  for (uint8_t i = 0; i < 3U; i++) {

    byte_cpy((uint8_t*)&decode_temp, &input[2U * i], 2);

    for (uint8_t j = 0; j < 3U; j++) {

      dummy = decode_temp & ( (uint16_t)0x1FU << (5U * j) );
      dummy = dummy >> (5U * j);
      if (dummy >= 16U) {
        dummy -= 32U;
      }
      diff[j + (3U * i)] = (int16_t)dummy;
    }
  }
}

/**
  * @brief  Copy source buffer in destination buffer.
  *
  * @param  destination      Destination buffer.(ptr)
  * @param  source           Source buffer.(ptr)
  *
  */
static void byte_cpy(uint8_t *destination, uint8_t *source, uint32_t len)
{
  uint32_t i;

  for ( i = 0; i < len; i++ ){
    destination[i] =  source[i];
  }

}

/**
  * @}
  *
  */

/**
  * @}
  *
  */