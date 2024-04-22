# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F411RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f411re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Read data

Read accelerometer and temperature sensor data in polling mode:

  - iis2iclx_read_data_polling.c

## Finite State Machine (FSM)

Program IIS2ICLX FSM to detect tilt events (read more [here](https://github.com/STMicroelectronics/STMems_Finite_State_Machine/blob/master/application_examples/iis2iclx/Tilt%20detection/README.md)):

  - iis2iclx_tilt.c

## Machine Learning Core (MLC)

Program IIS2ICLX MLC to detect tilt events (read more [here](https://github.com/STMicroelectronics/STMems_Machine_Learning_Core/blob/master/application_examples/iis2iclx/tilt_angle/README.md)):

  - iis2iclx_mlc_tilt_angle.c

