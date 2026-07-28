# Examples

This folder contains application examples for the NMH1000 magnetic sensor. The examples are tested with following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

## HW Setup
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)<br>
- FRDMSTBI-NMH1000 (https://www.st.com/en/evaluation-tools/frdmstbi-nmh1000.html)<br>
  FRDMSTBI-NMH1000 board setting: <br>
  - Connect Pins 1-2 of J7 and J8 jumpers.
  - Connect Pins 1-2 of J1 to configure the part in I2C mode.
  - Connect Pins 1-2 of J11 to route NMH1000 OUT signal to pin D7 on Arduino UNO headers.

# Examples Details
## Read Data

  1. nmh1000_read_data_polling.c:<br>
     Details: This example demonstrates how to configure data ready signaling in Auto Mode 
     for the NMH1000 sensor and how to read samples when the data‑ready flag is asserted.

## Read Events

  2. nmh1000_user_thresholds.c: <br>
     Details: This example demonstrates how to configure the NMH1000 sensor
     with a User Assert Threshold. It continuously prints the magnetic
     field measurements so the user can monitor the values in real time.
     When the magnetic field exceeds the configured threshold, the OUT
     pin goes high and the main program handles the interrupt by
     printing a message indicating that the User Threshold has been
     exceeded.

  3. nmh1000_magnetic_field_detect.c: <br>
     Details: This file shows an example of how to configure the NMH1000 in
     Auto-Mode and handle the OUT interrupt generated when a magnetic
     field is detected. Upon each interrupt occurrence, the main
     program reads the magnetic field value and prints it to the
     serial terminal.

  4. nmh1000_one_shot.c: <br>
     Details: This example demonstrates how to configure the NMH1000 sensor in 
     One Shot mode, and how to read samples when the data‑ready flag is asserted.


