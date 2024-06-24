# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lsm6dsv_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - lsm6dsv_read_data_polling.c

Read accelerometer sensor data on INT1 data ready:

  - lsm6dsv_read_data_irq.c

Read accelerometer and gyroscope sensor data from FIFO on FIFO threshold event:

  - lsm6dsv_fifo.c

## Program and use embedded digital functions

Program LSM6DSV to receive 6D orientation detection events on INT1:

  - lsm6dsv_six_d_position.c

Program LSM6DSV to receive single/double tap events on INT1:

  - lsm6dsv_single_double_tap.c

Program LSM6DSV to receive wakeup events on INT1:

  - lsm6dsv_wakeup.c

Program LSM6DSV to receive free-fall events on INT1:

  - lsm6dsv_free_fall.c

Program LSM6DSV Sensor Fusion Low Power (SFLP) to receive GBIAS, Gravity and Game rotation vectors:

  - lsm6dsv_sensor_fusion.c

## Sensor HUB

Program LSM6DSV to receive in FIFO accelerometer data as well as pressure and
magnetometer data from lps22df and lis2mdl sensors attached through Sensor HUB:

  - lsm6dsv_sensor_hub.c

