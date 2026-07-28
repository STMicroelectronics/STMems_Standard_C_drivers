# Examples

This folder contains application examples for NBP8FD4S battery pressure monitoring sensor. The examples are tested on following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

## HW Setup
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- FRDM-STBA-NBP8 (https://www.st.com/en/evaluation-tools/frdmstbanbp8xd.html)<br>
  FRDM-STBA-NBP8 board settings: <br>
  - Cut or bend pin 3 of connector 3 (J3-3) to prevent it from connecting to the Arduino header of the Nucleo board.
  - Add a fly wire between pin 3 of connector 3 (J3-3) and pin 3 of connector 5 (J5-3).

# Examples Details
## Read Data

  1. nbp8fd4s_read_pressure_data_interrupt.c:<br>
    Details: This example demonstrates how to configure the Data Ready interrupt in the NBP,
	     and read pressure, temperature and voltage data periodically when the interrupt occurs.

  2. nbp8fd4s_read_pressure_data_polling.c:<br>
    Details: This example demonstrates how to periodically trigger communication with the NBP
	     to read pressure, temperature and voltage. It also demonstrates how to handle
	     interrupt notifications.
  

