1 - Introduction
==================

This repository contains [platform independent drivers]( http://www.st.com/content/st_com/en/products/embedded-software/mems-and-sensors-software/drivers-for-mems/c-driver-mems.html ) for [STMicroelectronics](www.st.com/mems) sensors. Sensor drivers and examples were written in C programming language.

This repository contains two types of folders, identifiable using the following naming convention:

- folder that contains the *sensor drivers*, named  `xxxxxxx_STdC` where  `xxxxxxx` identifies the a sensor part number

- folder that contains the *demo project*, named  `_prj_XXXXXXX`where  `XXXXXXX` is the name of the ST evaluation board. 

## 1.a - Sensor driver folder structure   

Every *sensor driver* folder contains:

- `xxxxxxx_STdC\driver` : the C sensor driver (.h and .c) to be included in your project. Driver documentation can be generated using the [Doxigen](http://www.doxygen.org/) tool.
- `xxxxxxx_STdC\example`:  examples showing how to integrate the C driver in a project. They are written for [STM32 Microcontrollers](https://www.st.com/en/microcontrollers.html) using the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) tool, but they can be used as a guideline for every platform.
- README: additional info about the specific driver.

## 1.b - Demo project folder structure   

Every *demo project* folder contains a single configuration file for the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) tool named `_prj_XXXXXXX\XXXXXX.ioc`where  `XXXXXXX` is the name of the ST evaluation board. 

Using the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) tool ( configured with the related MCU Package )  and the `.ioc` file, it is possible to create a project in which you can easily run the examples available in each *sensor drivers* folder.

------



# 2 - Integration details
The driver is platform-independent, you only need to define the two functions for read and write transactions from the sensor hardware bus (ie. SPI or I²C).

## 2.a Source code integration

- Include in your project the driver files of Sensor (.h and .c) located in the `xxxxxxx_STdC\driver`folder of the corresponding product

- Define in your code the read and write functions that use the I²C or SPI platform driver like the following:

```c
/** Please note that is MANDATORY: return 0 -> no Error.**/
int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
```

- Declare and initialize the structure of device interface:

```c
xxxxxxx_ctx_t dev_ctx; /** xxxxxxx is the used part number **/
dev_ctx.write_reg = platform_write;
dev_ctx.read_reg = platform_read;
```

- If needed by the platform read and write functions, initialize the handle parameter:

```c
dev_ctx.handle = &platform_handle;
```


## 2.b Required properties

> * A standard C language compiler for the target MCU
> * A C library for the target MCU and the desired interface (ie. SPI, I2C) 

------



# 3 - Running Examples

They are written for [STM32 Microcontrollers](https://www.st.com/en/microcontrollers.html) using [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) tool, but they can be used as a guideline for every platform.

## 3.a Using a STMicroelectronics evaluation boards

In case of using the supported STMicroelectronics evaluation boards the examples file(.c) can run without applying any modifications (as is). 

In order to do that, please follow the following steps:

1. 
   Download and install [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html)  tool and the related MCU package (i.e. [STM32CubeF4](http://www.st.com/en/development-tools/stm32cubemx.html) for *NucleoF411* and *STEVAL_MKI109V3*).
2. Open the  `.ioc` configuration file associated to the selected evaluation board with the [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html) tool. The  `.ioc` configuration files for the supported evaluation boards can be found in the related ST evaluation board *demo project* folder. 
3. Generate the project using the [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html) tool and select your preferred IDE / Toolchain.
4. Add to your project the STMicroelectronics sensor driver. Driver files are located in the *sensor drivers* folder at`xxxxxxx_STdC\driver\xxxxxxx_reg.c(.h)` where  `xxxxxxx` identifies the sensor part number.
5. Add to your project the example source file (.c) that you are interested in. Example files are located in the *sensor drivers* folder at `xxxxxxx_STdC\example` where  `xxxxxxx` identifies a sensor part number.
6. Uncomment the selected board definition in section `/* STMicroelectronics evaluation boards definition */` in the selected example file (.c).     
7. Add the call to the example function inside the `while(1)` loop in the `main()` function the`main.c` file automatically generated by the  [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html) tool.
8. Enjoy :-)

## 3.b Running examples using different hardware

If a different MCU is used, please follow these steps:

1. Add to your project the STMicroelectronics sensor driver.  Driver files are located in the *sensor drivers* folder at`xxxxxxx_STdC\driver\xxxxxxx_reg.c(.h)` where  `xxxxxxx` identifies a sensor part number.
2. Add to your project the example source file (.c) that you are interested in. Example files are located in the *sensor drivers* folder at `xxxxxxx_STdC\example` where  `xxxxxxx` identifies the sensor part number.
3. Comment all the definitions of the boards in section `/* STMicroelectronics evaluation boards definition */ in the selected example file(.c). 
4. Add the call to the example function inside the `while(1)` loop in your`main()` function. 
5. Modify in the selected example file (.c) the hardware-related functions:

   - `platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,uint16_t len)`
   - `platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)`
   - if needed add/replace the hardware-related functions reported in the example file.
6. Enjoy :-)

------



**More Information: [http://www.st.com](http://st.com/MEMS)**

**Copyright (C) 2018 STMicroelectronics**

