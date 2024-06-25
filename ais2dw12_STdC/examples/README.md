# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - ais2dw12_self_test.c

## Read data

Read accelerometer and temperature sensor data in polling mode:

  - ais2dw12_read_data_polling.c

Read accelerometer and temperature sensor data from FIFO on FIFO threshold event:

  - ais2dw12_read_fifo.c

## Program and use embedded digital functions

Program AIS2DW12 to receive activity events:

  - ais2dw12_activity.c

Program AIS2DW12 to receive free fall events:

  - ais2dw12_free_fall.c

Program AIS2DW12 to receive 6D orientation detection events:

  - ais2dw12_orientation.c

Program AIS2DW12 to receive wakeup from sleep events:

  - ais2dw12_wake_up.c

