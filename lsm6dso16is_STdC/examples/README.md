# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lsm6dso16is_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in both interrupt (drdy) and polling mode:

  - lsm6dso16is_read_data_polling.c
  - lsm6dso16is_read_data_drdy.c

## Sensor HUB

Program LSM6DSO16IS to receive accelerometer and gyrometer data as well as
magnetometer and pressure data from lis2mdl/lps22df sensors attached through
Sensor HUB in drdy mode:

  - lsm6dso16is_sensor_hub.c

## ISPU

Program LSM6DSO16IS ISPU to compute the norm of the three axes of the accelerometer
(read more [here](https://github.com/STMicroelectronics/ispu-examples/blob/master//ism330is_lsm6dso16is/norm/README.md):

  - lsm6dso16is_ispu_norm.c

