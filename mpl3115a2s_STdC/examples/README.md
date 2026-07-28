# Examples

This folder contains application examples for MPL3115A2S pressure/altimeter sensor. The examples are tested on following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

## HW Setup
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- DISCOVERY_SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)
- FRDM-STBA-P3115 (https://www.st.com/en/evaluation-tools/frdmstbc-p3115.html)<br>

### FRDMSTBC-P3115 board settings
I2C communication:
- Install jumpers on pins 1-2 of J7 and J8 to enable I2C0.

INT signals (NUCLEO-F401RE):
- INT1: Connect a fly-wire between J3-6 and J4-6 to reroute the MPL3115 INT1 signal to PC0 (EXTI).
- INT2: No HW modification required. Signal connected by default to PA10 (EXTI).

*Note: The examples listed below use INT2 by default (when enabled). To use INT1 instead, some 
       hardware modifications are needed on the FRDMSTBC-P3115 board, see INT signals summary.*

*Note: The examples below require floating-point support for printf to display floating-point values on the terminal.*
  To enable this feature, navigate to:
  - Right-click project → Properties → C/C++ Build → Settings → Tool Settings → MCU/MPU Settings, then enable:
  ```Use float with printf from newlib-nano (-u _printf_float)```
  
# Examples Details
## Read Data

  1. mpl3115a2s_read_pressure_data_polling.c:<br>
    Details: This example demonstrates how to configure MPL3115A2S in
    polling mode to poll data ready event and read pressure, temperature
    samples on data ready event.

  2. mpl3115a2s_read_altimeter_data_polling.c:<br>
    Details: This example demonstrates how to configure MPL3115A2S in
    ALT mode to poll data ready event and read altitude, temperature
    samples on data ready event.
  
  3. mpl3115a2s_read_pressure_data_interrupt.c:<br>
    Details: This example demonstrates how to configure MPL3115A2S data ready
    in Interrupt mode and read samples on data ready interrupt occurrence.

  4. mpl3115a2s_read_pressure_data_oneshot.c:<br>
    Details: This example demonstrates how to configure MPL3115A2S in
    one-shot mode and read one sample of pressure measurement on OST flag.

  5. mpl3115a2s_read_pressure_threshold_int.c:<br>
     Details: This example demonstrates how to configure a pressure threshold
     event. To enable threshold event detection, the MPL3115A2S requires both
     the P_TGT (Pressure Target) and P_WND (Pressure Window) registers to be
     configured.
     The pressure threshold interrupt is enabled and is triggered when the
     measured pressure either enters or exits the window defined around the
     target pressure value.
     Pressure data is acquired automatically at the programmed Sample Time (ST)
     interval, while event detection is monitored by polling the STATUS register.
