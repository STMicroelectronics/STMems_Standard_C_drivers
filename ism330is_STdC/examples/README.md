# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - ism330is_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in both interrupt (drdy) and polling mode:

  - ism330is_read_data_polling.c
  - ism330is_read_data_drdy.c

## ISPU

Program ISM330IS ISPU to compute the norm of the three axes of the accelerometer
(read more [here](https://github.com/STMicroelectronics/ispu-examples/blob/master//ism330is_lsm6dso16is/norm/README.md):

  - ism330is_ispu_norm.c

