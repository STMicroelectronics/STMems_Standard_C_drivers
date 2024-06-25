# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Read data

Read pressure and temperature sensor data in both polling and drdy mode:

  - ilps22qs_read_data_polling.c

Read pressure and temperature sensor data from FIFO on FIFO threshold event:

  - ilps22qs_fifo.c

## Read AH_QVAR data

Program ILPS22QS to read AH_QVAR data:

  - ilps22qs_qvar_read_data_polling.c

