# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Read data

Read gyroscope sensor data in polling mode:

  - a3g4250d_read_data_polling.c

Read gyroscope sensor data from FIFO on FIFO threshold event:

  - a3g4250d_fifo_read.c

## Program and use embedded digital functions

Program A3G4250D to receive wakeup from sleep events:

  - a3g4250d_wake_up.c

