# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109D (https://www.st.com/en/evaluation-tools/steval-mki109d.html)
- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html) **[OBSOLETE]**
- NUCLEO-F401RE (https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

Please note that STEVAL-MKI109V3 board is OBSOLETE. Replacement with STEVAL-MKI109D is recommended.

## Read data

Read accelerometer and temperature sensor data in interrupt mode:

  - iis3dwb10is_read_data.c
  - iis3dwb10is_read_data_16b.c

Read accelerometer and temperature sensor data from FIFO on FIFO threshold event:

  - iis3dwb10is_fifo_read.c
  - iis3dwb10is_fifo_read_low_power.c

