# NUCLEO-H503RB platform

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

## How to run a driver example

Once a simple project has been generated, it needs to be customized in order to be able to run a driver example. Below there is a short description of the steps required to run [$STDC_PATH/lsm6dsv16x_STdC/examples/lsm6dsv_fifo_irq.c](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsv16x_STdC/examples/lsm6dsv16x_fifo_irq.c).

**Note:** $STDC_PATH <ins>is the root of STdC drivers/examples code</ins>

### Customize source code

1. Open the *$STDC_PATH/_prj_Nucleo_H503RB/Core/Src/main.c* file previously generated to edit it

2. Define the prototypes for both the example routine and its interrupt handler:

```c
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* add prototypes */
void lsm6dsv16x_fifo_irq(void);
void lsm6dsv16x_fifo_irq_handler(void);

/* USER CODE END PFP */
```

3. Call the example routine inside the main loop:

```c
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* call example main routine */
    lsm6dsv16x_fifo_irq();
  }
  /* USER CODE END 3 */
```

4. Define the general interrupt callback (which overwrites the default one, since it is declared as \__weak) and call the example interrupt handler:

```c
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* overwrite default interrupt callback */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  lsm6dsv16x_fifo_irq_handler();
}

/* USER CODE END 0 */
```

### Modify and build the project

1. Add the [$STDC_PATH/lsm6dsv16x_STdC/driver/lsm6dsv16x_reg.c](https://github.com/STMicroelectronics/lsm6dsv16x-pid/blob/main/lsm6dsv16x_reg.c) driver and [$STDC_PATH/lsm6dsv16x_STdC/examples/lsm6dsv_fifo_irq.c](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsv16x_STdC/examples/lsm6dsv16x_fifo_irq.c) example to the project.

2. Add the *$STDC_PATH/lsm6dsv16x_STdC/driver* directory to the compiler include path, e.g. :

```make
CFLAGS += -I $STDC_PATH/lsm6dsv16x_STdC/driver
```

3. Add NUCLEO_H503RB to the list of preprocessor enabled macros for the compiler, e.g :

```make
CFLAGS += -D NUCLEO_H503RB
```

4. Add the [$STDC_PATH/_prj_Nucleo_H503RB/i3c_api.c](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/_prj_Nucleo_H503RB/i3c_api.c) module to the project as well as the path to the [$STDC_PATH/_prj_Nucleo_H503RB/i3c_api.h](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/_prj_Nucleo_H503RB/i3c_api.h) header file.

```make
CFLAGS += -I $STDC_PATH/_prj_Nucleo_H503RB/
```

### Visualize example output messages

Open a serial port emulator and configure it as 921600 8n1:

<p align="center">
  <img src="./serial_port.png" />
</p>

**More information:**
  - [NUCLEO-H503RB](https://www.st.com/en/evaluation-tools/nucleo-h503rb.html)
  - [STM32H5](http://st.com/STM32H5)

**Copyright (C) 2024 STMicroelectronics**
