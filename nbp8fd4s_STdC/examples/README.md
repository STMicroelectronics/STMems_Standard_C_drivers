# Examples

This folder contains application examples for NBP8FD4S battery pressure monitoring sensor. The examples are tested on following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

## HW Setup
- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- FRDM-STBA-NBP8 (https://www.st.com/en/evaluation-tools/frdmstbanbp8xd.html)<br>
  FRDM-STBA-NBP8 board settings: <br>
  - Cut or bend pin 3 of connector J3 (J3-3) to prevent it from connecting to the Arduino header of the Nucleo board.
  - Add a fly wire between pin 3 of connector J3 (J3-3) and pin 3 of connector J5 (J5-3).

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
  - Select PB6 and configure it as GPIO_Output. PB6 will be used as CS.
  - Update PB6 User Label to: CS_up (very important as label needs to match in the source code).
  - GPIO output level: High
  - Select PA8 and configure it as GPIO_EXTI8.
  - Make sure that the User Label for PA8 is M_INT3 (very important as label needs to match in the source code).
- In System Core, go to the NVIC configuration:
  - Make sure the EXTI line[9:5] interrupts is Enabled.
 
Save the .ioc file and generate the project code.<br>
Build and program the firmware onto the target board.

*Important: The SPI interface is disabled by default in the .ioc project and must be enabled and configured in STM32CubeMX before use. Incorrect values for CPOL, CPHA, or the baud-rate prescaler may prevent communication with the NBP8FD4S sensor.*

<b>Main file. Example integration</b>

For integration of the NBP8FD4S examples, please refer to the NUCLEO-F401RE project README available at the following [link](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/_prj_Nucleo_F401RE).

*Note: Ensure that main.c includes a call to the data-ready interrupt handler from within HAL_GPIO_EXTI_Callback(), as shown below:
```
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	nbp8fd4s_drdy_irq_handler();
}
```

*Note: Before building the project, ensure the following macro is enabled:.*<br>
```#define NUCLEO_F401RE_SPI    /* little endian */```

*Note: The examples below require floating-point support for printf to display floating-point values on the terminal.*
  To enable this feature, navigate to:
  - Right-click project → Properties → C/C++ Build → Settings → Tool Settings → MCU/MPU Settings, then enable:<br>
  ```Use float with printf from newlib-nano (-u _printf_float)```

# Examples Details
## Read Data

  1. nbp8fd4s_read_pressure_data_interrupt.c:<br>
    Details: This example demonstrates how to configure the Data Ready interrupt in the NBP,
	     and read pressure, temperature and voltage data periodically when the interrupt occurs.

  2. nbp8fd4s_read_pressure_data_polling.c:<br>
    Details: This example demonstrates how to periodically trigger communication with the NBP
	     to read pressure, temperature and voltage. It also demonstrates how to handle
	     interrupt notifications.
  

