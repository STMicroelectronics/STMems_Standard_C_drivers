# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lis2ds12_self_test.c

## Read data

Read accelerometer and temperature sensor data in polling mode:

  - lis2ds12_read_data_polling.c

Read 8-bit and 14-bit accelerometer module:

  - lis2ds12_read_8bit_module.c
  - lis2ds12_read_14bit_module.c

Read accelerometer and temperature sensor data from FIFO on FIFO threshold event:

  - lis2ds12_multi_read_fifo.c

## Program and use embedded digital functions

Program LIS2DS12 to receive free fall events:

  - lis2ds12_free_fall.c

Program LIS2DS12 to receive 6D orientation detection events:

  - lis2ds12_orientation_6d.c

Program LIS2DS12 to receive single/double tap events:

  - lis2ds12_tap_single.c
  - lis2ds12_tap_double.c

Program LIS2DS12 to receive activity detection events:

  - lis2ds12_activity.c

Program LIS2DS12 to receive wakeup from sleep events:

  - lis2ds12_wake_up.c

