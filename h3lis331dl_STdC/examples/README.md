# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Read data

Read accelerometer sensor data in both interrupt and polling mode:

  - h3lis331dl_read_data_polling.c
  - h3lis331dl_read_data_drdy.c

## Program and use embedded digital functions

Program H3LIS331DL to receive wakeup from sleep events:

  - h3lis331dl_wake_up.c

