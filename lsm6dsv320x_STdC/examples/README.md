# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)
- NUCLEO-H503RB (https://www.st.com/en/evaluation-tools/nucleo-h503rb.html)

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - lsm6dsv320x_read_data_polling.c

Read accelerometer sensor data on INT1 data ready:

  - lsm6dsv320x_read_data_drdy.c

Read accelerometer and gyroscope sensor data from FIFO on FIFO threshold event:

  - lsm6dsv320x_fifo.c

## Program and use embedded digital functions

Program LSM6DSV320X to receive High-g wakeup events and data on INT1:

  - lsm6dsv320x_hg_wakeup.c

## Finite State Machine (FSM)

Program LSM6DSV320X FSM to detect *glance* and *de-glance* gestures typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/glance_detection/lsm6dsv320x/README.md)):

  - lsm6dsv320x_fsm_glance.c

Program LSM6DSV320X FSM to detect 4D position recognition typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/fourd_orientation_detection/lsm6dsv320x/README.md)):

  - lsm6dsv320x_fsm_fourd.c

## Machine Learning Core (MLC)

Program LSM6DSV320X MLC for gym activity recognition used on wearable device, such a smartwatch or a wristband. (read more [here](https://github.com/STMicroelectronics/STMems_Machine_Learning_Core/blob/master/application_examples/lsm6dsv320x/gym_activity_recognition/README.md)):

  - lsm6dsv320x_mlc_gym.c

