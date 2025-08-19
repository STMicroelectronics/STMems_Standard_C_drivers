# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lsm6dsv16bx_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - lsm6dsv16bx_read_data_polling.c

Read accelerometer and gyroscope sensor data from FIFO on FIFO threshold event (polling mode):

  - lsm6dsv16bx_fifo.c

## Program and use embedded digital functions

Program LSM6DSV16BX to act as a step counter:

  - lsm6dsv16bx_pedometer.c

Program LSM6DSV16BX to receive wake up events on INT1:

  - lsm6dsv16bx_wake_up.c

Program LSM6DSV16BX to receive activity/inactivity events:

  - lsm6dsv16bx_activity.c

Program LSM6DSV16BX Sensor Fusion Low Power (SFLP) to receive GBIAS, Gravity and Game rotation vectors:

  - lsm6dsv16bx_sensor_fusion.c

## Read AH_QVAR data

Program LSM6DSV16BX to read AH_QVAR data:

  - lsm6dsv16bx_qvar_read_data_polling.c

## Finite State Machine (FSM)

Program LSM6DSV16BX FSM to detect *glance* and *de-glance* gestures typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/glance_detection/lsm6dsv16bx/README.md)):

  - lsm6dsv16bx_fsm_glance.c

Program LSM6DSV16BX FSM to detect 4D position recognition typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/fourd_orientation_detection/lsm6dsv16bx/README.md)):

  - lsm6dsv16bx_fsm_fourd.c

## Machine Learning Core (MLC)

Program LSM6DSV16BX MLC for activity recognition on wrist used on wearable device, such a smartwatch or a wristband. (read more [here](https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/activity_recognition_for_wrist/lsm6dsv16bx/README.md)):

  - lsm6dsv17bx_mlc_gym.c

