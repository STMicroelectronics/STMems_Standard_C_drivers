# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

Read STHS34PF80 sensor data (Tobject/Tambient/Tpresence/Tmotion/Tamb_shock data) in polling mode:

  - sths34pf80_tmos_data_polling.c

Read STHS34PF80 sensor flags (Presence/Motion/Ambient-shock flags) in data ready mode:

  - sths34pf80_tmos_data_drdy.c

Program STHS34PF80 sensor to get Presence and Motion Detection events on INT1 pin:

  - sths34pf80_tmos_presence_detection.c

Program STHS34PF80 sensor to get Presence and Motion Detection events on INT1 pin and to reset the embedded algorithms when Ambient-shock events are detected:

  - sths34pf80_tmos_tamb_shock_reset.c

Program STHS34PF80 sensor to implement a software Presence Detection algorithm and to get events on INT1 pin (it requires the usage of a platform timer):

  - sths34pf80_tmos_sleep_app.c
