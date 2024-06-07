# Examples

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - ism330dhcx_self_test.c

## Read data

Read accelerometer, gyroscope and temperature sensor data in polling mode:

  - ism330dhcx_read_data_polling.c

## Sensor HUB

Program ISM330DHCX to receive in FIFO accelerometer, gyroscope and timestamp data as well as magnetometer data from iis2mdc sensor attached through Sensor HUB:

  - ism330dhcx_sensor_hub_iis2mdc_fifo_timestamp.c

Program ISM330DHCX to receive accelerometer and gyroscope data as well as magnetometer data from iis2mdc sensor attached through Sensor HUB:

  - ism330dhcx_sensor_hub_iis2mdc_fifo_timestamp.c

## Finite State Machine (FSM)

Program ISM330DHCX FSM to detect if device is falling down (read more [here](https://github.com/STMicroelectronics/STMems_Finite_State_Machine/blob/master/application_examples/ism330dhcx/Freefall%20detection/README.md)):

  - ism330dhcx_fsm_freefall.c

## Machine Learning Core (MLC)

Program ISM330DHCX MLC for 6D position recognition (read more [here](https://github.com/STMicroelectronics/STMems_Machine_Learning_Core/blob/master/application_examples/ism330dhcx/6D%20position%20recognition/README.md)):

  - ism330dhcx_mlc_six_d_position.c

