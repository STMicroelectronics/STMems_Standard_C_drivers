# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109D (https://www.st.com/en/evaluation-tools/steval-mki109d.html)
- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html) **[OBSOLETE]**
- NUCLEO-F401RE (https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)
- NUCLEO-H503RB (https://www.st.com/en/evaluation-tools/nucleo-h503rb.html)

*Nucleo H503RB board has been used to run the examples on I3C bus. All the examples except the sensor_hub one have been run on a [X-NUCLEO-IKS4A1 shield](https://www.st.com/en/ecosystems/x-nucleo-iks4a1.html) with a LSM6DSV16X device on DIL24. In all these cases, the shield has been configured in Mode 1 (Standard Mode). For the sensor_hub example case the shield is configured in Mode 3 (lsm6dsv16x SensorHub Mode), and the DIL24 is not required.*

Please note that STEVAL-MKI109V3 board is OBSOLETE. Replacement with STEVAL-MKI109D is recommended.

## Self Test (ST)

Run the device Self Test procedure:

  - lsm6dsv16b_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - lsm6dsv16b_read_data_polling.c

Read accelerometer and gyroscope sensor data from FIFO on FIFO threshold event (polling mode):

  - lsm6dsv16b_fifo.c

