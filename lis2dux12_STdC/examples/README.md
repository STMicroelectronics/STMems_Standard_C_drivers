# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lis2dux12_self_test.c

## Read data

Read accelerometer and temperature sensor data in both polling and drdy mode:

  - lis2dux12_read_data_polling.c
  - lis2dux12_read_data_drdy.c

Read accelerometer and temperature sensor data from FIFO on FIFO threshold event:

  - lis2dux12_read_fifo.c

## Program and use embedded digital functions

Program LIS2DUX12 to receive free fall events on INT1:

  - lis2dux12_free_fall.c

Program LIS2DUX12 to receive step counter events from FIFO:

  - lis2dux12_pedo_fifo.c

Program LIS2DUX12 to receive step counter events on INT1:

  - lis2dux12_pedometer.c

Program LIS2DUX12 to receive 6D orientation detection events on INT1:

  - lis2dux12_sixd.c

Program LIS2DUX12 to receive single/double/triple tap events on INT1:

  - lis2dux12_tap.c

Program LIS2DUX12 to receive tilt detection events on INT1:

  - lis2dux12_tilt.c

Program LIS2DUX12 to receive wakeup from sleep events on INT1:

  - lis2dux12_wakeup.c

## Finite State Machine (FSM)

Program LIS2DUX12 FSM to detect *glance* and *de-glance* gestures typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/glance_detection/lis2dux12/README.md)):

  - lis2dux12_fsm_glance.c

## Machine Learning Core (MLC)

Program LIS2DUX12 MLC to recognize user activity optimized for mobile devices (read more [here](https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/activity_recognition_for_mobile/lis2dux12/README.md)):

  - lis2dux12_mlc_activity_mobile.c

