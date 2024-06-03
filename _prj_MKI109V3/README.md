# STEVAL-MKI109V3 platform

This folder contains a simple STEVAL MKI109V3 CubeMX .ioc project file, which may be used to set up a basic system based on SPI with interrupt and UART capability and that can be used to test SPI-based examples.

Please read the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) documentation to understand how to automatically generate a simple software project that can be selected among several different toolchains, from basic Makefile to more complex ones, like IAR EWARM and Keil MDK-ARM.

## How to run a driver example

Once a simple project has been generated, it needs to be customized in order to be able to run a driver example. Below there is a short description of the steps required to run *$STDC_PATH/lsm6dsv16x_STdC/examples/lsm6dsv16x_fifo_irq.c*.

### Customize source code

1. Open the *$STDC_PATH/_prj_MKI109V3/Src/main.c* file previously generated to edit it

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
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

3. Add STEVAL_MKI109V3 to the list of preprocessor enabled macros for the compiler, e.g :

```make
CFLAGS += -D STEVAL_MKI109V3
```

### Visualize example output messages

Open a serial port emulator and configure it as 115200 8n1:

<p align="center">
  <img src="./serial_port.png" />
</p>

**More information:**
  - [STEVAL-MKI109V3](https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
  - [STM32F401VE](https://www.st.com/en/microcontrollers-microprocessors/stm32f401ve.html)

**Copyright (C) 2024 STMicroelectronics**
