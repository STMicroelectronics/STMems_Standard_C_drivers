# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - ais2ih_self_test.c

## Read data

Read accelerometer sensor data in polling mode:

  - ais2ih_read_data_polling.c

Read accelerometer sensor data from FIFO on FIFO threshold event:

  - ais2ih_multi_read_fifo.c

## Program and use embedded digital functions

Program AIS2IH to receive free fall events:

  - ais2ih_free_fall.c

Program AIS2IH to receive 6D orientation detection events:

  - ais2ih_orientation.c

Program AIS2IH to receive single/double tap events:

  - ais2ih_tap_single.c
  - ais2ih_tap_double.c

Program AIS2IH to receive wakeup from sleep events:

  - ais2ih_wake_up.c

