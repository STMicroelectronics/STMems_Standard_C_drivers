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

  - asm330lhhx_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - asm330lhhx_read_data_polling.c

Read accelerometer sensor data on INT1 data ready:

  - asm330lhhx_read_data_interrupt.c

Read accelerometer, gyroscope and temperature sensor data with accelerometer offset correction:

  - asm330lhhx_read_data_simple_offset.c

Read accelerometer, gyroscope and temperature sensor data with also timestamp:

  - asm330lhhx_read_data_simple_timestamp.c

Read accelerometer and gyroscope sensor data from FIFO on FIFO threshold event:

  - asm330lhhx_read_fifo_simple.c

## Program and use embedded digital functions

Program ASM330LHHX to receive 6D and 4D orientation detection events:

  - asm330lhhx_orientation_6d_4d.c

Program ASM330LHHX to receive wakeup events:

  - asm330lhhx_wake_up.c

Program ASM330LHHX to receive free-fall events:

  - asm330lhhx_free_fall.c

Program ASM330LHHX to receive activity/inactivity events:

  - asm330lhhx_activity.c

## Finite State Machine (FSM)

Program ASM330LHHX FSM to detect *motion* and *stationary* states (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/motion_stationary_detection/asm330lhhx/README.md)):

  - asm330lhhx_fsm_motion.c

Program ASM330LHHX FSM to detect shake gesture recognition (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/shake_detection/asm330lhhx/README.md)):

  - asm330lhhx_shake.c

## Machine Learning Core (MLC)

Program ASM330LHHX MLC for vehicle stationary/motion recognition used on vehicles. (read more [here](https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/vehicle_stationary_detection/asm330lhhx/README.md)):

  - asm330lhhx_vehicle_stationary.c

