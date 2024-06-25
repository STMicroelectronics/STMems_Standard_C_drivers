# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Read data

Read accelerometer. gyroscope and temperature sensor data in polling mode
with or without timestamp:

  - asm330lhh_read_data_polling.c
  - asm330lhh_read_timestamp.c

Read accelerometer. gyroscope and temperature sensor data from FIFO on FIFO threshold event:

  - asm330lhh_fifo_read.c

## Program and use embedded digital functions

Program ASM330LHH to receive free fall events:

  - asm330lhh_free_fall.c

Program ASM330LHH to receive 6D orientation detection events:

  - asm330lhh_orientation.c

Program ASM330LHH to receive wakeup from sleep events:

  - asm330lhh_wake_up.c

