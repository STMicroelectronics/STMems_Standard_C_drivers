/**
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef MAG_ANOMALIES_DETECTION_FSM_H
#define MAG_ANOMALIES_DETECTION_FSM_H

#ifdef __cplusplus
  extern "C" {
#endif

#include <stdint.h>

#define LSM6DSOX_MAG_ANOMALIES_DETECTION_SENSOR_NUM 1

#ifndef MEMS_CONF_SHARED_TYPES
#define MEMS_CONF_SHARED_TYPES

#define MEMS_CONF_ARRAY_LEN(x) (sizeof(x) / sizeof(x[0]))


/*
 * MEMS_CONF_SHARED_TYPES format supports the following operations:
 * - MEMS_CONF_OP_TYPE_TYPE_READ: read the register at the location specified
 *   by the "address" field ("data" field is ignored)
 * - MEMS_CONF_OP_TYPE_TYPE_WRITE: write the value specified by the "data"
 *   field at the location specified by the "address" field
 * - MEMS_CONF_OP_TYPE_TYPE_DELAY: wait the number of milliseconds specified by
 *   the "data" field ("address" field is ignored)
 * - MEMS_CONF_OP_TYPE_TYPE_POLL_SET: poll the register at the location
 *   specified by the "address" field until all the bits identified by the mask
 *   specified by the "data" field are set to 1
 * - MEMS_CONF_OP_TYPE_TYPE_POLL_RESET: poll the register at the location
 *   specified by the "address" field until all the bits identified by the mask
 *   specified by the "data" field are reset to 0
 */

 struct mems_conf_name_list {
  const char *const *list;
  uint16_t len;
};

enum {
  MEMS_CONF_OP_TYPE_READ = 1,
  MEMS_CONF_OP_TYPE_WRITE = 2,
  MEMS_CONF_OP_TYPE_DELAY = 3,
  MEMS_CONF_OP_TYPE_POLL_SET = 4,
  MEMS_CONF_OP_TYPE_POLL_RESET = 5
};

struct mems_conf_op {
  uint8_t type;
  uint8_t address;
  uint8_t data;
};

struct mems_conf_op_list {
  const struct mems_conf_op *list;
  uint32_t len;
};

#endif /* MEMS_CONF_SHARED_TYPES */

#ifndef MEMS_CONF_METADATA_SHARED_TYPES
#define MEMS_CONF_METADATA_SHARED_TYPES

struct mems_conf_application {
  char *name;
  char *version;
};

struct mems_conf_result {
  uint8_t code;
  char *label;
};

enum {
  MEMS_CONF_OUTPUT_CORE_HW = 1,
  MEMS_CONF_OUTPUT_CORE_EMB = 2,
  MEMS_CONF_OUTPUT_CORE_FSM = 3,
  MEMS_CONF_OUTPUT_CORE_MLC = 4,
  MEMS_CONF_OUTPUT_CORE_ISPU = 5
};

enum {
  MEMS_CONF_OUTPUT_TYPE_UINT8_T = 1,
  MEMS_CONF_OUTPUT_TYPE_INT8_T = 2,
  MEMS_CONF_OUTPUT_TYPE_CHAR = 3,
  MEMS_CONF_OUTPUT_TYPE_UINT16_T = 4,
  MEMS_CONF_OUTPUT_TYPE_INT16_T = 5,
  MEMS_CONF_OUTPUT_TYPE_UINT32_T = 6,
  MEMS_CONF_OUTPUT_TYPE_INT32_T = 7,
  MEMS_CONF_OUTPUT_TYPE_UINT64_T = 8,
  MEMS_CONF_OUTPUT_TYPE_INT64_T = 9,
  MEMS_CONF_OUTPUT_TYPE_HALF = 10,
  MEMS_CONF_OUTPUT_TYPE_FLOAT = 11,
  MEMS_CONF_OUTPUT_TYPE_DOUBLE = 12
};

struct mems_conf_output {
  char *name;
  uint8_t core;
  uint8_t type;
  uint16_t len;
  uint8_t reg_addr;
  char *reg_name;
  uint8_t num_results;
  const struct mems_conf_result *results;
};

struct mems_conf_output_list {
  const struct mems_conf_output *list;
  uint16_t len;
};

struct mems_conf_mlc_identifier {
  uint8_t fifo_tag;
  uint16_t id;
  char *label;
};

struct mems_conf_mlc_identifier_list {
  const struct mems_conf_mlc_identifier *list;
  uint16_t len;
};

#endif /* MEMS_CONF_METADATA_SHARED_TYPES */

static const char *const lsm6dsox_mag_anomalies_detection_format_version = "2.0";

static const char *const lsm6dsox_mag_anomalies_detection_description = NULL;

static const struct mems_conf_application lsm6dsox_mag_anomalies_detection_application = {
    .name = "Configuration Converter Tool",
    .version = "1.0"
};

static const char *const lsm6dsox_mag_anomalies_detection_date = NULL;

/* Sensor names */

static const char *const lsm6dsox_mag_anomalies_detection_names_0[] = {
    "lsm6dsox"
};

static const struct mems_conf_name_list lsm6dsox_mag_anomalies_detection_name_lists[LSM6DSOX_MAG_ANOMALIES_DETECTION_SENSOR_NUM] = {
    { .list = lsm6dsox_mag_anomalies_detection_names_0, .len = (uint16_t)MEMS_CONF_ARRAY_LEN(lsm6dsox_mag_anomalies_detection_names_0) }
};


/** Configurations **/

static const struct mems_conf_op lsm6dsox_mag_anomalies_detection_conf_0[] = {
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x10, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x11, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x01, .data = 0x80 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x04, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x05, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x5F, .data = 0x5B },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x46, .data = 0x01 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x47, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x0A, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x0B, .data = 0x01 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x0C, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x0E, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x0F, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x10, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x17, .data = 0x40 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x02, .data = 0x11 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x08, .data = 0x7A },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x01 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x01 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x04 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x02, .data = 0x41 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x08, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x50 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x0E },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x0A },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x3C },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x02 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x23 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x02 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x05 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x22 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x04, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x05, .data = 0x01 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x17, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x01, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x01, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x02, .data = 0x3F },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x07, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x08, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x09, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x0A, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x0B, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x0C, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x0D, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x0E, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x10, .data = 0x40 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x11, .data = 0x40 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x12, .data = 0x04 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x13, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x14, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x15, .data = 0xFF },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x16, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x17, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x18, .data = 0xE0 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x19, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x56, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x57, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x58, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x59, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x5A, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x5B, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x5C, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x5D, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x5E, .data = 0x02 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x5F, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x6F, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x70, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x71, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x72, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x73, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x74, .data = 0x00 },
 { .type = MEMS_CONF_OP_TYPE_WRITE, .address = 0x75, .data = 0x00 }
};

static const struct mems_conf_op_list lsm6dsox_mag_anomalies_detection_confs[LSM6DSOX_MAG_ANOMALIES_DETECTION_SENSOR_NUM] = {
  { .list = lsm6dsox_mag_anomalies_detection_conf_0, .len = (uint32_t)MEMS_CONF_ARRAY_LEN(lsm6dsox_mag_anomalies_detection_conf_0) }
};

/* Outputs */

static const struct mems_conf_output_list lsm6dsox_mag_anomalies_detection_output_lists[LSM6DSOX_MAG_ANOMALIES_DETECTION_SENSOR_NUM] = {
  { .list = NULL, .len = 0 }
};

/* MLC identifiers */

static const struct mems_conf_mlc_identifier_list lsm6dsox_mag_anomalies_detection_mlc_identifier_lists[LSM6DSOX_MAG_ANOMALIES_DETECTION_SENSOR_NUM] = {
  { .list = NULL, .len = 0 }
};

#ifdef __cplusplus
}
#endif

#endif /* MAG_ANOMALIES_DETECTION_FSM_H */

