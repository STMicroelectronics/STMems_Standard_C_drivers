# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - ism330bx_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - ism330bx_read_data_polling.c

Read accelerometer and gyroscope sensor data from FIFO on FIFO threshold event (polling mode):

  - ism330bx_fifo.c

## Program and use embedded digital functions

Program ISM330BX to act as a step counter:

  - ism330bx_pedometer.c

Program ISM330BX to receive wake up events on INT1:

  - ism330bx_wake_up.c

Program ISM330BX to receive activity/inactivity events:

  - ism330bx_activity.c

Program ISM330BX Sensor Fusion Low Power (SFLP) to receive GBIAS, Gravity and Game rotation vectors:

  - ism330bx_sensor_fusion.c

## Read AH_QVAR data

Program ISM330BX to read AH_QVAR data:

  - ism330bx_qvar_read_data_polling.c
## Finite State Machine (FSM)

Program ISM330BX FSM to detect 4D position recognition typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/fourd_orientation_detection/ism330bx/README.md)):

  - ism330bx_fsm_fourd.c

## Machine Learning Core (MLC)

Program ISM330BX MLC to monitor device vibration in industrial applications. (read more [here](https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/vibration_monitoring/ism330bx/README.md)):

  - ism330bx_mlc_vibration_monitoring.c

