# Examples

This folder contains application examples for FXPS7250A4S absolute pressure sensor family. 
The examples are tested on following boards but, since the drivers are platform independent, 
they can be easily ported to any other platform: 

- NUCLEO-F401RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html)
- FRDM7X-INTERFACE (https://www.st.com/en/evaluation-tools/frdm7x-interface.html)
- BRKTSTBAPDI7250 (https://estore.st.com/en/products/evaluation-tools/product-evaluation-tools/mems-and-sensors-eval-boards/brktstbapx7250s.html)


## Analog only output sensor examples

  1. fxps7250_analog_output_read.c:<br>
     Details: Reads analog output from FXPS7250A4S using ADC interrupt mode.
     ADC is configured for single conversion and each conversion is triggered 
     by the user program. It converts ADC samples to pressure (kPa) and prints 
     results at ~10 Hz using a basic state machine.
       - Please follow the additional steps below to integrate this example 
         and its associated drivers into your project.


  2. fxps7250_analog_output_stream.c:<br>
     Details: Continuously reads FXPS7250A4S analog output using ADC interrupt mode.
     ADC must be configured in continuous mode to generate samples at a constant rate.
     Applies 16× oversampling (averaging) to improve measurement precision.
     Converts samples to pressure (kPa) and prints results at ~5 Hz.
       - Please follow the additional steps below to integrate this example 
         and its associated drivers into your project.


<br>
<br>
  

## Additional Steps for Running ADC Examples for Analog Output Sensors

The following link provides guidance on how to generate a new STM32 project using STM32CubeMX based on the provided .ioc file:
https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/_prj_Nucleo_F401RE

### ADC Integration for Analog Output Sensors
When evaluating a sensor that provides analog output only, an ADC must be enabled and configured in the MCU project. This can be accomplished by regenerating the project using the supplied .ioc file.
Follow the steps below:
 1. Open the provided .ioc file for NUCLEO-F401RE in STM32CubeMX.
 2. Navigate to the Analog configuration section.
 3. Enable ADC1 and add Channel IN0.
 4. ADC Configuration Parameters<br>
    Configure the ADC with the following settings:
    - Channel: IN0
    - Resolution: 12 bits
    - Continuous Conversion Mode:
        Example 1: Disabled, 
        Example 2: Enabled
    - End of Conversion Selection: 
        Example 1: ADC_EOC_SINGLE_CONV, 
        Example 2: ADC_EOC_SEQ_CONV <br>
      Note: All other ADC parameters can be left at their default values or adjusted by the user as needed.
 5. NVIC Configuration
    - Enable the ADC1 global interrupt in the NVIC settings.
 6. Save and regenerate the source code in STM32CubeMX to include the ADC drivers.

### Customizing the Source Code for the Analog Output Example
To run the FXPS7250 analog output example, the application flow differs from the previous digital sensor examples.

The following steps must now be followed:
 1. Open the generated source file:
      ```
      $STDC_PATH/_prj_Nucleo_F401/Src/main.c
      Note: all of these steps are done in the main.c file.
 2. Call all required initialization functions to configure the sensor and the ADC:
      ```
      fxps7250_analog_example();<br>
 3. Execute the analog sensor example routine inside the main loop to process measurements:
      ```
      fxps7250_analog_stream();<br>
 4. The ADC interrupt handler must be implemented and linked to the HAL callback:
      ```
      /* overwrite default interrupt callback */
      void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
      {
        fxps7250_adc_irq_handler(hadc);
      }
