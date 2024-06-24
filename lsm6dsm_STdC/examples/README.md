# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lsm6dsm_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - lsm6dsm_read_data_polling.c

Read accelerometer, gyroscope and temperature sensor data in polling mode generating and displayng a timestamp value as well:

  - lsm6dsm_timestamp.c

Read accelerometer and gyroscope sensor data from FIFO on FIFO threshold event in polling mode:

  - lsm6dsm_fifo_stream_to_fifo.c
  - lsm6dsm_read_fifo.c

## Program and use embedded digital functions

Program LSM6DSM to receive activity/inactivity events:

  - lsm6dsm_activity.c

Program LSM6DSM to receive single/double tap events:

  - lsm6dsm_tap_double.c
  - lsm6dsm_tap_single.c

Program LSM6DSM to receive 6D orientation events:

  - lsm6dsm_orientation.c

Program LSM6DSM to receive tilt and wrist tilt events:

  - lsm6dsm_tilt.c
  - lsm6dsm_wrist_tilt.c

Program LSM6DSM to receive wakeup events:

  - lsm6dsm_wake_up.c

Program LSM6DSM to receive free-fall events:

  - lsm6dsm_free_fall.c

## Sensor HUB

Program LSM6DSM to receive accelerometer and gyrometer data as well as magnetometer
and/or pressure data from lis2mdl/lps22hb sensors attached through Sensor HUB
in polling mode:

  - lsm6dsm_sens_hub_lis2mdl.c
  - lsm6dsm_sens_hub_lis2mdl_lps22hb.c
  - lsm6dsm_sens_hub_lps22hb.c

## Sensor HUB and FIFO

Program LSM6DSM to receive in FIFO accelerometer and gyrometer data as well as magnetometer
data from lis2mdl sensors attached through Sensor HUB using FIFO threshold events on INT1:

  - lsm6dsm_sens_hub_fifo_lis2mdl.c
  - lsm6dsm_sens_hub_fifo_lis2mdl_passthrough.c

Program LSM6DSM to receive in FIFO accelerometer and gyrometer data as well as pressure
data from lps22hb (and lis2mdl) sensors attached through Sensor HUB (polling mode):

  - lsm6dsm_sens_hub_fifo_lps22hb.c
  - lsm6dsm_sens_hub_fifo_lps22hb_lis2mdl.c

