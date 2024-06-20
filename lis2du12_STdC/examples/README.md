# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lis2du12_self_test.c

## Read data

Read accelerometer and temperature sensor data in both polling and drdy mode:

  - lis2du12_read_data_polling.c
  - lis2du12_read_data_drdy.c

Read accelerometer and temperature sensor data from FIFO on FIFO threshold event:

  - lis2du12_read_data_fifo.c

## Program and use embedded digital functions

Program LIS2DU12 to receive free fall events on INT1:

  - lis2du12_free_fall.c

Program LIS2DU12 to receive 6D orientation detection events on INT1:

  - lis2du12_sixd.c

Program LIS2DU12 to receive single/double/triple tap events on INT1:

  - lis2du12_tap.c

