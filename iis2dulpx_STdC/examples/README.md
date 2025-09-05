# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - iis2dulpx_self_test.c

## Read data

Read accelerometer and temperature sensor data on drdy event:

  - iis2dulpx_read_data_drdy.c

Read accelerometer and temperature sensor data from FIFO on FIFO threshold event:

  - iis2dulpx_read_fifo.c

## Program and use embedded digital functions

Program IIS2DULPX to receive step counter events from FIFO:

  - iis2dulpx_pedo_fifo.c

Program IIS2DULPX to receive step counter events on INT1:

  - iis2dulpx_pedometer.c

## Read AH_QVAR data

Program IIS2DULPX to read AH_QVAR data on INT1:

  - iis2dulpx_qvar_read_data.c

## Finite State Machine (FSM)

Program IIS2DULPX FSM to detect 4D position recognition typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/fourd_orientation_detection/iis2dulpx/README.md)):

  - iis2dulpx_fsm_fourd.c

## Machine Learning Core (MLC)

Program IIS2DULPX MLC to monitor device vibration in industrial applications. (read more [here](https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/vibration_monitoring/iis2dulpx/README.md)):

  - iis2dulpx_mlc_vibration_monitoring.c

