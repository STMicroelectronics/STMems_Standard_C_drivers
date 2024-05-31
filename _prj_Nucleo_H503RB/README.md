# Nucleo H503RG platform

This folder contains a simple STM32H5 CubeMX .ioc project file, which declares the minimal things necessary to set up a basic system based on I3C with interrupt and UART capability and that can be used to test I3C-based examples.

Please read the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) documentation to understand how to automatically generate a simple software project that can be selected among several different toolchains, from basic Makefile to more complex ones, like IAR EWARM and Keil MDK-ARM.

The folder includes also a basic set of APIs based on [STM32H5 HAL](https://www.st.com/en/embedded-software/stm32cubeh5.html) that can be used to abstract I3C bus communication:

  ```c
  - int32_t i3c_rstdaa(I3C_HandleTypeDef *handle);
  - int32_t i3c_set_bus_frequency(I3C_HandleTypeDef *handle, uint32_t i3c_freq);
  - int32_t i3c_setdasa(I3C_HandleTypeDef *handle, uint8_t addr, uint8_t *cccdata, uint16_t len);
  - int32_t i3c_write(I3C_HandleTypeDef *handle, uint16_t addr, uint16_t reg, uint8_t *pdata, uint16_t len);
  - int32_t i3c_read(I3C_HandleTypeDef *handle, uint16_t addr, uint16_t reg, uint8_t *pdata, uint16_t len);
  ```

These APIs are already used in some of the STdC driver examples, hence the user is expected to manually edit the selected project adding both *i3c_api.c* module and the path of the *i3c_api.h* include file to properly build them.

**More information:**
  - [NUCLEO-H503RB](https://www.st.com/en/evaluation-tools/nucleo-h503rb.html)
  - [STM32H5](http://st.com/STM32H5)

**Copyright (C) 2024 STMicroelectronics**
