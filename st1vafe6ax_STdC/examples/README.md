# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F411RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f411re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - st1vafe6ax_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - st1vafe6ax_read_data_polling.c

Read accelerometer and gyroscope sensor data from FIFO on FIFO threshold event (polling mode):

  - st1vafe6ax_fifo.c

## Read vAFE data

Read vAFE data using data ready on INT2 pin

  - st1vafe6ax_vafe_read_data.c

