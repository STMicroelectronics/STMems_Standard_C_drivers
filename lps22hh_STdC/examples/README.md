# Examples

This folder contains application examples.
The examples refer to following boards but, since the drivers are platform independent,
they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Read data

Read pressure and temperature sensor data in both polling and drdy mode:

  - lps22hh_read_data_polling.c
  - lps22hh_read_data_drdy.c

Read pressure and temperature sensor data from FIFO on FIFO threshold event:

  - lps22hh_read_fifo_irq.c

