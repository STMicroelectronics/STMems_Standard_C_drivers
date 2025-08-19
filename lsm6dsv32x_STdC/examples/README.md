# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lsm6dsv32x_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - lsm6dsv32x_read_data_polling.c

Read accelerometer and gyroscope sensor data from FIFO on FIFO threshold event (polling and interrupt mode):

  - lsm6dsv32x_fifo.c
  - lsm6dsv32x_fifo_irq.c

## Program and use embedded digital functions

Program LSM6DSV32X to receive 6D orientation detection events on INT1:

  - lsm6dsv32x_six_d_position.c

Program LSM6DSV32X to receive single/double tap events on INT1:

  - lsm6dsv32x_single_double_tap.c

Program LSM6DSV32X Sensor Fusion Low Power (SFLP) to receive GBIAS, Gravity and Game rotation vectors:

  - lsm6dsv32x_sensor_fusion.c

## Finite State Machine (FSM)

Program LSM6DSV32X FSM to detect *glance* and *de-glance* gestures typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/fourd_orientation_detection/lsm6dsv32x/README.md)):

  - lsm6dsv32x_fsm_glance.c

Program LSM6DSV32X FSM to detect 4D position recognition typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/glance_detection/lsm6dsv32x/README.md)):

  - lsm6dsv32x_fsm_fourd.c

## Machine Learning Core (MLC)

Program LSM6DSV32X MLC for gym activity recognition used on wearable device, such a smartwatch or a wristband. (read more [here](https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/gym_activity_recognition/lsm6dsv32x/README.md)):

  - lsm6dsv32x_mlc_gym.c

