Index
=======
            * Introduction
            * Integration details
            * Required properties
            * Copyright


Introduction
==============
This repository contains platform-independent drivers and examples for STMicroelectronics MEMS sensors. The sensors drivers and examples are listed by product. Every product folder contains:   

### 'driver' folder
This folder contains the driver files of Mems Sensor (.h and .c) to be included in your project.
Driver documentation can be generated using Doxigen tool.

### 'examples' folder:
This folder contains examples showing how to integrate the Standard C driver in a project. They are written for STM32 Microcontroller using CubeMX API, but they can be used as a guideline for every platform.

Integration details
=====================
The driver is platform-independent, you only need to define the two functions for writing to and reading from Mems hardware bus (SPI or I2C).

### Source code integration

> * Include in your project the driver files of Mems Sensor (.h and .c) located in the 'driver' folder of the desider product
> * Define in your code the read and write functions that use the I2C or SPI platform driver like the following:

>         int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
>         int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)

> * Initialize the structure of device interface:

>           dev_ctx.write_reg = platform_write;
>           dev_ctx.read_reg = platform_read;

> * If needed by the platform read and write functions, initialize the handle parameter:

>           dev_ctx.handle = &platform_handle;

> 
### Required properties

> * A standard C compiler for the desired target
> * A C library for the desired target and the desired interface (SPI, I2C) 

More Information
=================

[http://st.com](http://st.com/MEMS)

Copyright
===========
Copyright (C) 2018 STMicroelectronics

