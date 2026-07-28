# Examples

This folder contains application examples for FXLS8961AF 3-axis low-g accelerometer. The examples are tested on following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

## HW Setup
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- STEVAL-MKI109D (https://www.st.com/en/evaluation-tools/steval-mki109d.html)
- DISCOVERY_SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)
- FRDM-STBA-A8961 (https://www.st.com/en/evaluation-tools/frdm-stba-a8961.html) <br>
  FRDM-STBA-A8961 board settings for I2C communication: <br>
  - Connect Pins 1-2 of J7 and J8 jumpers.
  - Connect Pins 2-3 of SW1.
  - Connect Pins 2-3 of SW2.

# Examples Details
## Read Data

  1. fxls8961af_read_data_polling.c:<br>
    Details: This example demonstrates how to configure FXLS8961AF in
    polling mode to poll data ready event and read sensor samples on data
    ready event.

  2. fxls8961af_read_data_fifo.c:<br>
    Details: This example demonstrates how to configure FXLS8961AF in FIFO
    mode and read buffered samples when configured FIFO_DEPTH is reached.
  
  3. fxls8961af_read_data_interrupt.c: <br>
    Details: This example demonstrates how to configure FXLS8961AF in Interrupt
    mode and read samples on data ready interrupt occurrence.

## Read Events

  4. fxls8961af_sdcd_motion_detect.c: <br>
    Details: This example demonstrates how to configure FXLS8961AF SDCD function
    along with auto-wake/sleep feature to detect wake-up sensor motion event and
    autonomously move sensor to sleep mode when no motion detected.

  5. fxls8961af_sdcd_freefall_detect.c: <br>
    Details: This example demonstrates how to configure FXLS8961AF SDCD WT function
    to configure sensor for detecting freefall event.

  6. fxls8961af_sdcd_tap_detect.c: <br>
    Details: This example demonstrates how to configure FXLS8961AF SDCD OT function
    to configure sensor for detecting tap or transient event.
