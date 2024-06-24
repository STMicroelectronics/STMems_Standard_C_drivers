# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - lsm6dsr_read_data_polling.c

Read accelerometer, gyroscope and temperature sensor data in polling mode
applying accelerometer offset correction:

  - lsm6dsr_offset.c

Read accelerometer, gyroscope and temperature sensor data in polling mode
generating and displayng a timestamp value as well:

  - lsm6dsr_timestamp.c

Read accelerometer and gyroscope sensor data, both compressed and uncompressed, from FIFO on FIFO threshold event in polling mode:

  - lsm6dsr_compressed_fifo.c
  - lsm6dsr_multi_read_fifo.c

## Program and use embedded digital functions

Program LSM6DSR to receive activity/inactivity events:

  - lsm6dsr_activity.c

Program LSM6DSR to receive single/double tap events:

  - lsm6dsr_tap.c

Program LSM6DSR to receive 4D and 6D orientation events:

  - lsm6dsr_orientation.c

Program LSM6DSR to detect and count step events in FIFO:

  - lsm6dsr_fifo_pedo.c

Program LSM6DSR to receive tilt events:

  - lsm6dsr_tilt.c

Program LSM6DSR to receive wakeup events:

  - lsm6dsr_wake_up.c

Program LSM6DSR to receive free-fall events:

  - lsm6dsr_free_fall.c

