# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F411RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f411re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lsm6dsox_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in both interrupt (drdy) and polling mode:

  - lsm6dsox_read_data_polling.c
  - lsm6dsox_read_data_drdy.c

Read accelerometer, gyroscope and temperature sensor data in polling mode applying accelerometer offset correction:

  - lsm6dsox_offset.c

Read accelerometer, gyroscope and temperature sensor data in polling mode generating and displayng a timestamp value as well:

  - lsm6dsox_timestamp.c

Read accelerometer and gyroscope sensor data, both compressed and uncompressed, from FIFO on FIFO threshold event in polling mode:

  - lsm6dsox_fifo.c
  - lsm6dsox_fifo_compressed.c

## Program and use embedded digital functions

Program LSM6DSOX to receive activity/inactivity events:

  - lsm6dsox_activity.c

Program LSM6DSOX to receive single/double tap events:

  - lsm6dsox_tap.c

Program LSM6DSOX to receive 6D orientation events:

  - lsm6dsox_orientation.c

Program LSM6DSOX to detect and count step events:

  - lsm6dsox_pedometer.c

Program LSM6DSOX to detect and count step events in FIFO:

  - lsm6dsox_fifo_step.c

Program LSM6DSOX to receive tilt events:

  - lsm6dsox_tilt.c

Program LSM6DSOX to receive wakeup events:

  - lsm6dsox_wake_up.c

Program LSM6DSOX to receive free-fall events:

  - lsm6dsox_free_fall.c

## Sensor HUB

Program LSM6DSOX to receive accelerometer and gyrometer data as well as magnetometer data from lis2mdl sensors attached through Sensor HUB in polling mode:

  - lsm6dsox_sh_lis2mdl.c

Program LSM6DSOX to receive accelerometer and gyrometer data as well as magnetometer data from lis2mdl sensors attached through Sensor HUB using data ready events on INT1:

  - lsm6dsox_sh_lis2mdl_drdy.c

Program LSM6DSOX to receive accelerometer and gyrometer data as well as magnetometer data from lis2mdl sensors attached through Sensor HUB using MLC:

  - lsm6dsox_sh_lis2mdl_mlc.c

## Sensor HUB and FIFO

Program LSM6DSOX to receive in FIFO accelerometer and gyrometer data as well as magnetometer data from lis2mdl sensors attached through Sensor HUB using FIFO threshold events on INT1:

  - lsm6dsox_sh_fifo_lis2mdl.c

Program LSM6DSOX to receive in FIFO accelerometer and gyrometer data as well as pressure data from lps22hh sensors attached through Sensor HUB (polling mode):

  - lsm6dsox_sh_fifo_lps22hh.c

Program LSM6DSOX to receive in FIFO accelerometer, gyrometer and timestamp data as well as magnetometer data from lis2mdl sensors attached through Sensor HUB (polling mode):

  - lsm6dsox_sh_fifo_lis2mdl_timestamp.c

## Finite State Machine (FSM)

Program LSM6DSOX FSM to detect *glance* and *de-glance* gestures typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/STMems_Finite_State_Machine/blob/master/application_examples/lsm6dsox/Glance%20detection/README.md)):

  - lsm6dsox_fsm_glance.c

Program LSM6DSOX FSM with 7 consecutive ucf configurations to detect *glance*, *motion*, *no_motion*, *wakeup*, *pickpup*, *orientation*, *wrist_tilt* events:

  - lsm6dsox_fsm_raw.c

Program LSM6DSOX FSM to

  - lsm6dsox_fsm_sh_mag_anomalies_detection.c

## Machine Learning Core (MLC)

Program LSM6DSOX MLC for vibration monitoring recognition (read more [here](https://github.com/STMicroelectronics/STMems_Machine_Learning_Core/blob/master/application_examples/lsm6dsox/Vibration%20monitoring/README.md)):

  - lsm6dsox_mlc.c

## Multi Configuration

Program LSM6DSOX to receive wakeup, single-double tap, tilt, step detection, sixd position and free fall events as well as Yoga Pose events from MLC:

  - lsm6dsox_multi_conf.c
