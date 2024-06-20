# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lis331dlh_self_test.c

## Read data

Read accelerometer and temperature sensor data in polling mode:

  - lis331dlh_read_data_polling.c

## Program and use embedded digital functions

Program LIS331DLH to receive free fall events:

  - lis331dlh_free_fall.c

Program LIS331DLH to receive 6D orientation detection events:

  - lis331dlh_orientation.c

Program LIS331DLH to receive wakeup from sleep events:

  - lis331dlh_wake_up.c

