# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
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

## Finite State Machine (FSM)

Program LSM6DSV16X FSM to detect *glance* and *de-glance* gestures typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/glance_detection/st1vafe6ax/README.md)):

  - st1vafe6ax_fsm_glance.c

Program LSM6DSV16X FSM to detect 4D position recognition typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/fourd_orientation_detection/st1vafe6ax/README.md)):

  - st1vafe6ax_fsm_fourd.c

## Machine Learning Core (MLC)

Program LSM6DSV16X MLC for activity recognition used on wearable device, such a smartwatch or a wristband. (read more [here](https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/activity_recognition_for_wrist/st1vafe6ax/README.md)):

  - st1vafe6ax_mlc_activity_wrist.c

