# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Read data

Read accelerometer and temperature sensor data in polling mode:

  - iis2iclx_read_data_polling.c

## Finite State Machine (FSM)

Program IIS2ICLX FSM to detect tilt events (read more [here](https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/tilt_detection/iis2iclx/README.md)):

  - iis2iclx_tilt.c

## Machine Learning Core (MLC)

Program IIS2ICLX MLC to detect tilt events (read more [here](https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/tilt_angle/iis2iclx/README.md)):

  - iis2iclx_mlc_tilt_angle.c

