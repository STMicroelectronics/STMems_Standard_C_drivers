# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109D (https://www.st.com/en/evaluation-tools/steval-mki109d.html)
- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html) **[OBSOLETE]**
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

Please note that STEVAL-MKI109V3 board is OBSOLETE. Replacement with STEVAL-MKI109D is recommended.

## Self Test (ST)

Run the device Self Test procedure:

  - iis3dwb_self_test.c

## Read data

Read accelerometer and temperature sensor data in polling mode:

  - iis3dwb_read_data_polling.c

Read accelerometer and temperature sensor data from FIFO on FIFO threshold event:

  - iis3dwb_fifo.c

## Program and use embedded digital functions

Program IIS3DWB to receive wakeup from sleep events:

  - iis3dwb_wake_up.c

