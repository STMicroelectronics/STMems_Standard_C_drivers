# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

Read TMOS sensor data (Tamb/Presence/Motion flags) in both polling and drdy mode:

  - sths34pf80_tmos_data_polling.c
  - sths34pf80_tmos_data_drdy.c

Program STHS34PF80 to get Presence Detection event on INT1:

  - sths34pf80_tmos_presence_detection.c
