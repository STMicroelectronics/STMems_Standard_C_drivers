# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - iis2dh_self_test.c

## Read data

Read accelerometer and temperature sensor data in polling mode:

  - iis2dh_read_data_polling.c

Read accelerometer and temperature sensor data from FIFO on FIFO threshold event:

  - iis2dh_fifo_read.c

## Program and use embedded digital functions

Program LIS3DH to receive free fall events:

  - iis2dh_free_fall.c

Program LIS3DH to receive 6D orientation detection events:

  - iis2dh_orientation.c

Program LIS3DH to receive single/double tap events:

  - iis2dh_tap_single.c
  - iis2dh_tap_double.c

Program LIS3DH to receive wakeup from sleep events:

  - iis2dh_wake_up.c

