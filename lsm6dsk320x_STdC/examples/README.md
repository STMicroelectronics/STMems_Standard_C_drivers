# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109D (https://www.st.com/en/evaluation-tools/steval-mki109d.html)
- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html) **[OBSOLETE]**
- NUCLEO-F401RE (https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)
- NUCLEO-H503RB (https://www.st.com/en/evaluation-tools/nucleo-h503rb.html)

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - lsm6dsk320x_read_data_polling.c

Read accelerometer sensor data on INT1 data ready:

  - lsm6dsk320x_read_data_drdy.c

Read accelerometer and gyroscope sensor data from FIFO on FIFO threshold event:

  - lsm6dsk320x_read_fifo.c

## Program and use embedded digital functions

Program LSM6DSK320X to receive High-g wakeup events and data on INT1:

  - lsm6dsk320x_hg_wakeup.c

