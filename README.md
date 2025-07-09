![latest tag](https://img.shields.io/github/v/tag/STMicroelectronics/STMems_Standard_C_drivers.svg?color=brightgreen)

# 1 - Introduction

This repository contains examples of *low-level* [platform-independent drivers]( http://www.st.com/content/st_com/en/products/embedded-software/mems-and-sensors-software/drivers-for-mems/c-driver-mems.html ) for [STMicroelectronics](www.st.com/mems) sensors. Sensor drivers and examples are written in C programming language.

If you are using [STM32Cube packages](https://www.st.com/en/ecosystems/stm32cube.html), evaluate also the *hardware abstractions* [STM32Cube-compatible drivers](https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer/blob/master/README.md#stm32cube-bsp-components-drivers). 

The STMicroelectronics naming convention for driver repositories is:
 - `PARTNUMBER` (*e.g. [hts221](https://github.com/STMicroelectronics/hts221-pid)*) for *low-level platform-independent drivers*
 - `stm32-PARTNUMBER` (*e.g. [stm32-hts221](https://github.com/STMicroelectronics/stm32-hts221)*) for *hardware-abstracted STM32Cube-compatible drivers*

### 1.a - Repository structure

This repository contains two types of folders, identifiable using the following naming convention:

- folder that contains the *sensor drivers*, named  `xxxxxxx_STdC` where  `xxxxxxx` identifies the sensor part number
- folder that contains the *demo project*, named  `_prj_XXXXXXX` where  `XXXXXXX` is the name of the ST evaluation board

Another folder, named  `_resources`,  cannot be identified with the two types described above and contains *other useful resources* such as libraries and predefined device configurations used in some examples. In order to `clone` the complete content of this folder, use the command:

```git
git clone --recursive https://github.com/STMicroelectronics/STMems_Standard_C_drivers
```

### 1.b - Sensor driver folder structure

Every *sensor driver* folder contains:

- `xxxxxxx_STdC\driver` : this folder points to another repository which contains only the sensor driver files (.h and .c) to be included, or linked directly as a git submodule, in your project. Driver documentation can be generated using the [Doxygen](http://www.doxygen.org/) tool.
- `xxxxxxx_STdC\examples`:  examples showing how to integrate the C driver in a project. They are written for [STM32 Microcontrollers](https://www.st.com/en/microcontrollers.html) using the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) tool, but they can be used as a guideline for every platform.
- README: additional info about the specific driver.

### 1.c - Demo project folder structure

Every *demo project* folder contains a single configuration file for the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) tool named `_prj_XXXXXXX\XXXXXX.ioc` where  `XXXXXXX` is the name of the ST evaluation board.

Using the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) tool (configured with the related MCU package) and the `.ioc` file, it is possible to create a project in which you can easily run the examples available in each *sensor driver* folder.

------

# 2 - Integration details
The driver is platform-independent, you only need to define the two functions for read and write transactions from the sensor hardware bus (ie. SPI or I2C/I3C) and a platform dependent delay function, which is sometimes required by the underlying driver.

**Note**: A few devices integrate an extra bit in the communication protocol in order to enable multi read/write access, this bit must be managed in read and write functions defined by the user. Please refer to the read and write implementation in the reference examples.

### 2.a Source code integration

- Include in your project the driver files of the sensor (.h and .c) located in the `xxxxxxx_STdC\driver` folder of the corresponding product.

- Define in your code the read and write as well as the delay functions that use the I2C, I3C or SPI platform driver like the following:

```c
/** Please note that it is MANDATORY: return 0 -> no Error.**/
int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
void platform_delay(uint32_t ms);
```

- Declare and initialize the structure of the device interface:

```c
xxxxxxx_ctx_t dev_ctx; /** xxxxxxx is the used part number **/
dev_ctx.write_reg = platform_write;
dev_ctx.read_reg = platform_read;
dev_ctx.mdelay = platform_delay;
```

- If needed by the platform read and write functions, initialize the handle parameter:

```c
dev_ctx.handle = &platform_handle;
```

- Any additional initialization work has to be done in platform_init routine:

```c
void platform_init(void);
```

### 2.b Required properties

> * A standard C language compiler for the target MCU
> * A C library for the target MCU and the desired interface (i.e. SPI, I2C, I3C)

------

# 3 - Running examples

Examples are written for [STM32 Microcontrollers](https://www.st.com/en/microcontrollers.html) using the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) tool, but they can be used as a guideline for every platform.

### 3.a Using STMicroelectronics evaluation boards

When using the supported STMicroelectronics evaluation boards, the example file(.c) can be run without applying any modifications (as is).

In order to run that file, please follow these steps:

1. Download and install the [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html)  tool and the related MCU package (i.e. STM32F4 for *NucleoF401* and *STEVAL_MKI109V3* and STM32H5 for *NucleoH503*).
2. Open the  `.ioc` configuration file associated to the selected evaluation board with the [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html) tool. The  `.ioc` configuration files for the supported evaluation boards can be found in the related ST evaluation board *demo project* folder.
3. Generate the project using the [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html) tool and select your preferred IDE / toolchain.
4. Add to your project the STMicroelectronics sensor driver. Driver files are located in the *sensor drivers* folder at`xxxxxxx_STdC\driver\xxxxxxx_reg.c(.h)` where  `xxxxxxx` identifies the sensor part number.
5. Add to your project the example source file (.c) that you are interested in. Example files are located in the *sensor drivers* folder at `xxxxxxx_STdC\example` where  `xxxxxxx` identifies the sensor part number.
6. Uncomment the selected board definition in section `/* STMicroelectronics evaluation boards definition */` in the selected example file (.c).
7. Add the call to the example function inside the `while(1)` loop in the `main()` function of the `main.c` file automatically generated by the  [STM32CubeMX](http://www.st.com/en/development-tools/stm32cubemx.html) tool.
8. Open a terminal emulator to see messages and configure it as 115200 8n1 for *NucleoF401* and *STEVAL_MKI109V3* or 921600 8n1 for *NucleoH503*.
9. Enjoy :-)

More info is available in each specific provided example platform:

- [NUCLEO_F401RE](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/_prj_Nucleo_F401RE)
- [STEVAL_MKI109V3](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/_prj_MKI109V3)
- [NUCLEO_H503RB](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/_prj_Nucleo_H503RB)

*If you are using [STM32Cube packages](https://www.st.com/en/ecosystems/stm32cube.html), evaluate also the **hardware-abstracted** STM32Cube-compatible drivers specifically designed to be compatible with the STM32Cube. The complete list is provided [here](https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer/blob/master/README.md#stm32cube-bsp-components-drivers).*

### 3.b Running examples using different hardware

If a different MCU is used, please follow these steps:

1. Add the STMicroelectronics sensor driver to your project.  Driver files are located in the *sensor drivers* folder at`xxxxxxx_STdC\driver\xxxxxxx_reg.c(.h)` where  `xxxxxxx` identifies a sensor part number.
2. Add the example source file (.c) that interests you to your project. Example files are located in the *sensor drivers* folder at `xxxxxxx_STdC\example` where  `xxxxxxx` identifies the sensor part number.
3. Comment all the definitions of the boards in section `/* STMicroelectronics evaluation boards definition */ in the selected example file(.c).
4. Add the call to the example function inside the `while(1)` loop in your`main()` function.
5. Modify in the selected example file (.c) the hardware-related functions:

   - `platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp,uint16_t len)`
   - `platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)`
   - `void platform_delay(uint32_t ms)`
   - if needed, add/replace the hardware-related functions reported in the example file.
6. Enjoy :-)

------

**More information: [http://www.st.com](http://st.com/MEMS)**

**Copyright (C) 2018-2024 STMicroelectronics**

