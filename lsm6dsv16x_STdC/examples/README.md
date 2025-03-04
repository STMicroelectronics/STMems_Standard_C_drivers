# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)
- NUCLEO-H503RB (https://www.st.com/en/evaluation-tools/nucleo-h503rb.html)

*Nucleo H503RB board has been used to run the examples on I3C bus. All the examples except the sensor_hub one have been run on a [X-NUCLEO-IKS4A1 shield](https://www.st.com/en/ecosystems/x-nucleo-iks4a1.html) with a LSM6DSV16X device on DIL24. In all these cases, the shield has been configured in Mode 1 (Standard Mode). For the sensor_hub example case the shield is configured in Mode 3 (lsm6dsv16x SensorHub Mode), and the DIL24 is not required.*

## Self Test (ST)

Run the device Self Test procedure:

  - lsm6dsv16x_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - lsm6dsv16x_read_data_polling.c

Read accelerometer sensor data on INT1 data ready:

  - lsm6dsv16x_read_data_irq.c

Read accelerometer and gyroscope sensor data, both compressed and uncompressed, from FIFO on FIFO threshold event (polling and interrupt mode):

  - lsm6dsv16x_fifo.c
  - lsm6dsv16x_fifo_irq.c
  - lsm6dsv16x_compressed_fifo.c

Read step counter virtual sensor from FIFO:

  - lsm6dsv16x_fifo_stepcnt.c

## Program and use embedded digital functions

Program LSM6DSV16X to receive step detection events and data on INT1:

  - lsm6dsv16x_pedometer.c

Program LSM6DSV16X to receive significant motion detection events on INT1:

  - lsm6dsv16x_sig_mot.c

Program LSM6DSV16X to receive relative tilt detection events on INT1:

  - lsm6dsv16x_tilt.c

Program LSM6DSV16X to receive 6D orientation detection events on INT1:

  - lsm6dsv16x_six_d_position.c

Program LSM6DSV16X to receive single/double tap events on INT1:

  - lsm6dsv16x_single_double_tap.c

Program LSM6DSV16X to receive wakeup events on INT1:

  - lsm6dsv16x_wakeup.c

Program LSM6DSV16X to receive free-fall events on INT1:

  - lsm6dsv16x_free_fall.c

Program LSM6DSV16X Sensor Fusion Low Power (SFLP) to receive GBIAS, Gravity and Game rotation vectors:

  - lsm6dsv16x_sensor_fusion.c

## Sensor HUB

Program LSM6DSV16X to receive in FIFO accelerometer data as well as pressure and
magnetometer data from lps22df and lis2mdl sensors attached through Sensor HUB:

  - lsm6dsv16x_sensor_hub.c

## Read AH_QVAR data

Program LSM6DSV16X to read AH_QVAR data in polling mode:

  - lsm6dsv16x_qvar_read_data.c

## Finite State Machine (FSM)

Program LSM6DSV16X FSM to detect *glance* and *de-glance* gestures typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/STMems_Finite_State_Machine/blob/master/application_examples/lsm6dsv16x/Glance%20detection/README.md)):

  - lsm6dsv16x_fsm_glance.c

Program LSM6DSV16X FSM to detect 4D position recognition typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/STMems_Finite_State_Machine/blob/master/application_examples/lsm6dsv16x/FourD%20position%20recognition/README.md)):

  - lsm6dsv16x_fsm_fourd.c

## Machine Learning Core (MLC)

Program LSM6DSV16X MLC for gym activity recognition used on wearable device, such a smartwatch or a wristband. (read more [here](https://github.com/STMicroelectronics/STMems_Machine_Learning_Core/blob/master/application_examples/lsm6dsv16x/gym_activity_recognition/README.md)):

  - lsm6dsv16x_mlc_gym.c

