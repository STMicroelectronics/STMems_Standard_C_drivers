# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F411RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f411re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)
- NUCLEO-H503RB (https://www.st.com/en/evaluation-tools/nucleo-h503rb.html)

## Read data

Read pressure and temperature sensor data in polling mode:

  - ilps28qsw_read_data_polling.c

Read pressure sensor data from FIFO when FIFO threshold is reached:

  - ilps28qsw_fifo.c

Read pressure and AH_QVAR sensor data from FIFO in interleaved mode when FIFO threshold is reached:

  - ilps28qsw_fifo_interleaved_data.c

## Read AH_QVAR data

Program ILPS28QSW to read AH_QVAR data in polling mode:

  - ilps28qsw_ah_qvar_read_data_polling.c

Program ILPS28QSW to read pressure and AH_QVAR sensor data in interleaved mode:

  - ilps28qsw_ah_qvar_press_interleaved.c

