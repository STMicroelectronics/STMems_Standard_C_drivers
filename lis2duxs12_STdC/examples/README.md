# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F411RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f411re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lis2duxs12_self_test.c

## Read data

Read accelerometer and temperature sensor data in both polling and drdy mode:

  - lis2duxs12_read_data_polling.c
  - lis2duxs12_read_data_drdy.c

Read accelerometer and temperature sensor data from FIFO on FIFO threshold event:

  - lis2duxs12_read_fifo.c

## Program and use embedded digital functions

Program LIS2DUXS12 to receive free fall events on INT1:

  - lis2duxs12_free_fall.c

Program LIS2DUXS12 to receive step counter events from FIFO:

  - lis2duxs12_pedo_fifo.c

Program LIS2DUXS12 to receive step counter events on INT1:

  - lis2duxs12_pedometer.c

Program LIS2DUXS12 to receive 6D orientation detection events on INT1:

  - lis2duxs12_sixd.c

Program LIS2DUXS12 to receive single/double/triple tap events on INT1:

  - lis2duxs12_tap.c

Program LIS2DUXS12 to receive tilt detection events on INT1:

  - lis2duxs12_tilt.c

Program LIS2DUXS12 to receive wakeup from sleep events on INT1:

  - lis2duxs12_wakeup.c

## Read AH_QVAR data

Program LIS2DUXS12 to read AH_QVAR data on INT1:

  - lis2duxs12_qvar_read_data.c

## Finite State Machine (FSM)

Program LIS2DUXS12 FSM to detect *glance* and *de-glance* gestures typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/STMems_Finite_State_Machine/blob/master/application_examples/lis2duxs12/Glance%20detection/README.md)):

  - lis2duxs12_fsm_glance.c

## Machine Learning Core (MLC)

Program LIS2DUXS12 MLC to recognize user activity optimized for mobile devices (read more [here](https://github.com/STMicroelectronics/STMems_Machine_Learning_Core/blob/master/application_examples/lis2duxs12/activity_recognition_for_mobile/README.md)):

  - lis2duxs12_mlc_activity_mobile.c

