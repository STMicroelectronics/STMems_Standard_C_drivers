# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - asm330lhhxg1_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - asm330lhhxg1_read_data_polling.c

Read accelerometer sensor data on INT1 data ready:

  - asm330lhhxg1_read_data_interrupt.c

Read accelerometer, gyroscope and temperature sensor data with accelerometer offset correction:

  - asm330lhhxg1_read_data_simple_offset.c

Read accelerometer, gyroscope and temperature sensor data with also timestamp:

  - asm330lhhxg1_read_data_simple_timestamp.c

Read accelerometer and gyroscope sensor data from FIFO on FIFO threshold event:

  - asm330lhhxg1_read_fifo_simple.c

## Program and use embedded digital functions

Program ASM330LHHXG1 to receive 6D and 4D orientation detection events:

  - asm330lhhxg1_orientation_6d_4d.c

Program ASM330LHHXG1 to receive wakeup events:

  - asm330lhhxg1_wake_up.c

Program ASM330LHHXG1 to receive free-fall events:

  - asm330lhhxg1_free_fall.c

Program ASM330LHHXG1 to receive activity/inactivity events:

  - asm330lhhxg1_activity.c

## Finite State Machine (FSM)

Program ASM330LHHXG1 FSM to detect *motion* and *stationary* states (read more [here](https://github.com/STMicroelectronics/STMems_Finite_State_Machine/blob/master/application_examples/asm330lhhx/Motion-Stationary%20detection/README.md)):

  - asm330lhhxg1_fsm_motion.c

Program ASM330LHHXG1 FSM to detect shake gesture recognition (read more [here](https://github.com/STMicroelectronics/STMems_Finite_State_Machine/blob/master/application_examples/asm330lhhx/Shake%20detection/README.md)):

  - asm330lhhxg1_shake.c

## Machine Learning Core (MLC)

Program ASM330LHHXG1 MLC for vehicle stationary/motion recognition used on vehicles. (read more [here](https://github.com/STMicroelectronics/STMems_Machine_Learning_Core/blob/master/application_examples/asm330lhhx/Vehicle%20stationary%20detection/README.md)):

  - asm330lhhxg1_vehicle_stationary.c

