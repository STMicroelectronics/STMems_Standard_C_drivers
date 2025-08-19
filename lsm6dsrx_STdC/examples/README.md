# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lsm6dsrx_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data both interrupt (drdy) and polling mode:

  - lsm6dsrx_read_data_polling.c
  - lsm6dsrx_read_data_interrupt.c

Read accelerometer, gyroscope and temperature sensor data in polling mode applying accelerometer offset correction:

  - lsm6dsrx_read_data_simple_offset.c

Read accelerometer, gyroscope and temperature sensor data in polling mode generating and displayng a timestamp value as well:

  - lsm6dsrx_read_data_simple_timestamp.c

Read accelerometer and gyroscope sensor data, both compressed and uncompressed, from FIFO on FIFO threshold event in polling mode:

  - lsm6dsrx_compressed_fifo.c
  - lsm6dsrx_multi_read_fifo_simple.c

## Program and use embedded digital functions

Program LSM6DSRX to receive activity/inactivity events:

  - lsm6dsrx_activity.c

Program LSM6DSRX to receive single/double tap events:

  - lsm6dsrx_single_double_tap.c

Program LSM6DSRX to receive 4D and 6D orientation events:

  - lsm6dsrx_orientation_6d_4d.c

Program LSM6DSRX to detect and count step events in FIFO:

  - lsm6dsrx_fifo_pedo.c

Program LSM6DSRX to receive tilt events:

  - lsm6dsrx_tilt.c

Program LSM6DSRX to receive wakeup events:

  - lsm6dsrx_wake_up.c

Program LSM6DSRX to receive free-fall events:

  - lsm6dsrx_free_fall.c

## Finite State Machine (FSM)

Program LSM6DSRX FSM (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/free_fall_detection/lsm6dsrx/README.md)):

  - lsm6dsrx_fsm_freefall_detection.c

## Machine Learning Core (MLC)

Program LSM6DSRX MLC for 6D position recognition (read more [here](https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/6d_position_recognition/lsm6dsrx/README.md)):

  - lsm6dsrx_mlc_six_d_position.c

