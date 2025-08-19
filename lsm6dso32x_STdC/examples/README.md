# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lsm6dso32x_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data both interrupt (drdy) and polling mode:

  - lsm6dso32x_read_data_polling.c
  - lsm6dso32x_read_data_int.c

Read accelerometer, gyroscope and temperature sensor data in polling mode applying accelerometer offset correction:

  - lsm6dso32x_offset.c

Read accelerometer, gyroscope and temperature sensor data in polling mode generating and displayng a timestamp value as well:

  - lsm6dso32x_timestamp.c

Read accelerometer and gyroscope sensor data, both compressed and uncompressed, from FIFO on FIFO threshold event in polling mode:

  - lsm6dso32x_fifo.c
  - lsm6dso32x_compressed_fifo.c

## Program and use embedded digital functions

Program LSM6DSO32X to receive activity/inactivity events:

  - lsm6dso32x_activity.c

Program LSM6DSO32X to receive single/double tap events:

  - lsm6dso32x_single_double_tap.c

Program LSM6DSO32X to receive 6D orientation events:

  - lsm6dso32x_orientation.c

Program LSM6DSO32X to detect and count step events:

  - lsm6dso32x_pedometer.c

Program LSM6DSO32X to detect and count step events in FIFO:

  - lsm6dso32x_fifo_pedo.c

Program LSM6DSO32X to receive tilt events:

  - lsm6dso32x_tilt.c

Program LSM6DSO32X to receive wakeup events:

  - lsm6dso32x_wake_up.c

Program LSM6DSO32X to receive free-fall events:

  - lsm6dso32x_free_fall.c

## Finite State Machine (FSM)

Program LSM6DSO32X FSM with 7 consecutive ucf configurations to detect *glance*, *motion*, *no_motion*, *wakeup*, *pickpup*, *orientation*, *wrist_tilt* events:

  - lsm6dso32x_fsm.c

## Machine Learning Core (MLC)

Program LSM6DSO32X MLC for vibration monitoring recognition (read more [here](https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/vibration_monitoring/lsm6dso32x/README.md)):

  - lsm6dso32x_mlc.c

