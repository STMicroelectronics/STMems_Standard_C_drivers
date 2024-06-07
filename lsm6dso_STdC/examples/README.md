# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lsm6dso_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - lsm6dso_read_data_polling.c

Read accelerometer, gyroscope and temperature sensor data in polling mode applying accelerometer offset correction:

  - lsm6dso_offset.c

Read accelerometer, gyroscope and temperature sensor data in polling mode generating and displayng a timestamp value as well:

  - lsm6dso_timestamp.c

Read accelerometer and gyroscope sensor data, both compressed and uncompressed, from FIFO on FIFO threshold event in polling mode:

  - lsm6dso_fifo.c
  - lsm6dso_compressed_fifo.c

## Program and use embedded digital functions

Program LSM6DSO to receive activity/inactivity events:

  - lsm6dso_activity.c

Program LSM6DSO to receive single/double tap events:

  - lsm6dso_tap.c

Program LSM6DSO to receive 6D orientation events:

  - lsm6dso_orientation.c

Program LSM6DSO to detect and count step events:

  - lsm6dso_pedometer.c

Program LSM6DSO to detect and count step events in FIFO:

  - lsm6dso_fifo_pedo.c

Program LSM6DSO to receive tilt events:

  - lsm6dso_tilt.c

Program LSM6DSO to receive wakeup events:

  - lsm6dso_wake_up.c

Program LSM6DSO to receive free-fall events:

  - lsm6dso_free_fall.c

## Sensor HUB and FIFO

Program LSM6DSO to receive in FIFO accelerometer and gyrometer data as well as magnetometer data from lis2mdl sensors attached through Sensor HUB (polling mode):

  - lsm6dso_sensor_hub_lis2mdl.c

Program LSM6DSO to receive in FIFO accelerometer and gyrometer data as well as pressure data from lps22hh sensors attached through Sensor HUB (polling mode):

  - lsm6dso_sensor_hub_lps22hh.c

## Finite State Machine (FSM)

Program LSM6DSO FSM with 7 consecutive ucf configurations to detect *glance*, *motion*, *no_motion*, *wakeup*, *pickpup*, *orientation*, *wrist_tilt* events:

  - lsm6dso_fsm.c

