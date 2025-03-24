# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)
- NUCLEO-H503RB (https://www.st.com/en/evaluation-tools/nucleo-h503rb.html)

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - lsm6dsv80x_read_data_polling.c

Read accelerometer sensor data on INT1 data ready:

  - lsm6dsv80x_read_data_drdy.c

Read accelerometer and gyroscope sensor data from FIFO on FIFO threshold event:

  - lsm6dsv80x_fifo.c
