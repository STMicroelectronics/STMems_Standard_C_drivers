# Examples

This folder contains application examples for FXPS7115D4S absolute pressure sensor family. 
The examples are tested on following boards but, since the drivers are platform independent, 
they can be easily ported to any other platform: 

## HW Setup
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- STEVAL-MKI109D (https://www.st.com/en/evaluation-tools/steval-mki109d.html)
- SPC584B-DIS (https://www.st.com/en/evaluation-tools/spc584b-dis.html)
- FRDM7X-INTERFACE (https://www.st.com/en/evaluation-tools/frdm7x-interface.html)
- BRKTSTBAPX7250S (https://www.st.com/en/evaluation-tools/brktstbapx7250s.html)

### FRDM7X-INTERFACE board settings
Communication interface configuration:
- Configure SW1 according to the Interface Selection Table on the board to select the required interface: I²C or SPI.
- Note that SW1 allows to select Analog Output interface for analog output sensor variants (fxps7115_STdC).

INT signals (NUCLEO-F401RE):
- INT1: Connect a fly-wire between J3-3 and J3-6 to reroute the FLXS7115 INT1 signal to PA10 (EXTI).
  - ``` Note this step is required for examples that use interruptions.```


## SW Configuration
### NUCLEO-F401RE
To run the examples using SPI communication, the user must perform the following additional steps:
- Open the NUCLEO-F401RE project (.ioc file) from the GitHub repository in STM32CubeMX.
- Go to Connectivity and enable the SPI1 peripheral in Full-Duplex/Master Mode.

<b>SPI Configuration</b><br>
In SPI Parameter Settings:
- In <b>Clock Paramaters</b>, set the following:
  - Prescaler (for Baud Rate): 64
  - Clock Polarity (CPOL): LOW
  - Clock Phase (CPHA): 1 Edge

    *Keep all remaining parameters at their default values.*

- In GPIO Settings, make sure that the pin assignments are: 
  - PA5 → SPI1_SCK, 
  - PA6 → SPI1_MISO, 
  - PA7 → SPI1_MOSI,
  
  ```If STM32CubeMX automatically configures PB3 as SPI1_SCK, change the pin assignment to PA5.```

<b>GPIO Configuration</b>
- In System Core, go to the GPIO configuration:
  - Select PB6 and configure it as GPIO_Output. PB6 will be used as NSS.
  - Update PB6 User Label to: SPI1_NSS (very important as label needs to match in the source code).
  - GPIO Level: High
 
Save the .ioc file and generate the project code.<br>
Build and program the firmware onto the target board.

*Important: The SPI interface is disabled by default in the .ioc project and must be enabled and configured in STM32CubeMX before use. Incorrect values for CPOL, CPHA, or the baud-rate prescaler may prevent communication with the FXPS7115 sensor.*

*Note: Before building the project, ensure the following macro is enabled:.*<br>
```#define NUCLEO_F401RE_SPI    /* little endian */```

*Note: The examples below require floating-point support for printf to display floating-point values on the terminal.*
  To enable this feature, navigate to:
  - Right-click project → Properties → C/C++ Build → Settings → Tool Settings → MCU/MPU Settings, then enable:<br>
  ```Use float with printf from newlib-nano (-u _printf_float)```
       
       
## Digital (I2C and SPI) sensors - Read Data

  1. fxps7115_read_data_polling.c:<br>
     Details: This example demonstrates how to configure FXPS7115 in
     polling mode to poll data ready event and read sensor samples on data
     ready event.


  2. fxps7115_threshold_interrupt.c:<br>
     Details: This example demonstrates how to configure FXPS7115 to assert
     the INT pin when the pressure measurements cross user-defined high or
     low threshold levels. The sensor is configured with programmable
     thresholds and generates an interrupt event when the pressure exceeds
     the high threshold or falls below the low threshold.

