# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Read data

Read accelerometer sensor data from FIFO:

  - st1vafe3bx_read_xl_fifo.c

## Read vAFE data

Read vAFE data:

  - st1vafe3bx_read_data_polling.c
  - st1vafe3bx_read_data_drdy.c
  - st1vafe3bx_read_fifo.c

## Machine Learning Core (MLC)

Program ST1VAFE3BX MLC device to recognize 6D position (read more [here](https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/6d_position_recognition/st1vafe3bx/README.md)):

  - st1vafe3bx_mlc_six_d_position.c

