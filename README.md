
# [MIP](https://mipot.com/en/products/?cat=110) Series Firmware Example for Dual Core

## Overview
<p align="justify">
The 32001506 family features a dual core microcontroller in which one is dedicated to the radio stack (ARM Cortex M0+) and the ARM Cortex M4 is free for the customer application firmware. It is based on STM32WL55JC microcontroller.
</p>

### ARM Cortex M0+
<p align="justify">
The arm®Cortex®-M0+ is reserved for sub-GHz radio management and implements the radio stack required by the specific version of the 32001506. 
It uses the LPTIM (LPTIM1/LPTIM2) for stack timing requirements and 32 kB SRAM2 block as volatile working memory.
</p>

### ARM Cortex M4
<p align="justify">
The arm®Cortex®-M4 is fully available for user application together with all the peripherals not used by the radio part (all except sub-GHz radio components and LPTIM1/LPTIM2). 
It uses 32 kB SRAM1 block as volatile working memory and shares flash memory with the arm®Cortex®-M0+ core.
</p>

### IPCC - Inter-Processor Communication Controller
<p align="justify">
IPCC is used to perform bidirectional communication between cores. 
It is an ST Microcontroller proprietary inter-core communication controller. 
For details please refer to “RM0453 Reference manual - STM32WL5x advanced arm®-based 32-bit MCUs with sub-GHz radio solution”.
</p>

<p align="justify">
IPCC communication operates on a common RAM memory area shared between arm®Cortex®-M4 and arm®Cortex®-M0+. 
In 32001506 a 1 kB area is reserved starting from address 0x20008000 to address 0x200083FF. 
The IPCC shared memory is totally inside SRAM2 block, so it does not affect arm®Cortex®-M4 available RAM.
</p>


## About
For more information, please refer to the documents downloadable from the official [MIPOT](https://www.mipot.com) website. Registration is required.

## UART Bridge
<p align="justify">
Designed to test the capabilities of the dual core module,
this firmware forwards messages received on the serial line (UART/LPUART) to the M0+ core via IPCC and vice versa.
As an initial step, the microcontroller dynamically selects the active serial communication line upon receiving the first signal.
This design choice is specific to this example firmware, which primarily serves to illustrate the 32001506 module's functionalities, including IPCC communication and command execution. 


Build with [MIPOT](https://www.mipot.com) [30001506CEU](https://mipot.com/en/products/evaluation-kit/dev-kits/30001506ceu/) DevKit.
</p>


## Compatibility
 - [30001506AEU](https://mipot.com/en/products/evaluation-kit/dev-kits/30001506aeu/) - 868 MHz Wireless M-Bus
 - [30001506BEU](https://mipot.com/en/products/evaluation-kit/dev-kits/30001506beu/) - 868 MHz LoRaWAN
 - [30001506CEU](https://mipot.com/en/products/evaluation-kit/dev-kits/30001506ceu/) - 868 MHz LoRa Mipot
 - [30001506DEU](https://mipot.com/en/products/evaluation-kit/dev-kits/30001506deu/) - 868 MHz LoRa Modem
 - [30001506FEU](https://mipot.com/en/products/evaluation-kit/dev-kits/30001506feu/) - 868 MHz Dual Stack (LoRaWAN + LoRa Modem) 
 - [30001506BUS](https://mipot.com/en/products/evaluation-kit/dev-kits/30001506bus/) - 915 MHz LoRaWAN

## Board Overview
<p align="center">
  <img src="https://github.com/Mipot-Hi-Tech/mip_uart_bridge/blob/master/img/img0.png">
</p>


- Application
<div align="center">

| PAD           |Description    |                   |PAD                |Description     
|:---------     |:----------    |:---------         |:----------        |:----------                    
| `B3`          | UART TX       |                   | `A4`              | LPUART TX                          
| `B4`          | UART RX       |                   | `A5`              | LPUART RX              
| `B1`          | NWAKE         |                   | `A9`              | VDD (3.3V)  

</div>


- For programming
<div align="center">

| PAD           |Description    |                   |PAD                |Description     
|:---------     |:----------    |:---------         |:----------        |:----------                    
| `G5`          | NRST          |                   | `G7`              | SWCLK                          
| `G6`          | SWDIO         |                   | `G8`              | SWO   
           
</div>

For more information, please refer to the datasheet downloadable from the official [MIPOT](https://www.mipot.com) website.


## Step 1 - Import project and build
Open STM32CubeIDE, then `File --> Import`

<p align="center">
  <img src="https://github.com/Mipot-Hi-Tech/mip_uart_bridge/blob/master/img/img7.png">
</p>

In the window that opens, select `General --> Projects from Folder or Archive`

<p align="center">
  <img src="https://github.com/Mipot-Hi-Tech/mip_uart_bridge/blob/master/img/img8.png">
</p>

Click on `Directory` and select the project folder, then `Finish` 

<p align="center">
  <img src="https://github.com/Mipot-Hi-Tech/mip_uart_bridge/blob/master/img/img9.png">
</p>

Build the project ![img10](https://github.com/Mipot-Hi-Tech/mip_uart_bridge/blob/master/img/img10.png).

If it becomes necessary to debug the firmware, you need to set the `DEBUGGER_ON` to 1 in the `\Core\Inc\sys_conf.h` file. 

By default, it is set to 0 to reduce power consumption.

![img6](https://github.com/Mipot-Hi-Tech/mip_uart_bridge/blob/master/img/img6.png)

## Step 2 - Program the DevKit

<p align="center">
  <img src="https://github.com/Mipot-Hi-Tech/mip_uart_bridge/blob/master/img/img1.png">
</p>

Press ![img11](https://github.com/Mipot-Hi-Tech/mip_uart_bridge/blob/master/img/img11.png) to download the firmware to the board.

## Step 3 - Hardware setup

<p align="center">
  <img src="https://github.com/Mipot-Hi-Tech/mip_uart_bridge/blob/master/img/img2.png">
</p>

## Step 4 - Software Setup

#### Start LoRaMiP GUI

#### Select the COM port to wich the device is connected (highlighted in green)

<p align="center">
  <img src="https://github.com/Mipot-Hi-Tech/mip_uart_bridge/blob/master/img/img3.png">
</p>

#### Application overview

<p align="center">
  <img src="https://github.com/Mipot-Hi-Tech/mip_uart_bridge/blob/master/img/img4.png">
</p>

#### Communication test

<p align="center">
  <img src="https://github.com/Mipot-Hi-Tech/mip_uart_bridge/blob/master/img/img5.png">
</p>

![#66ffff](https://placehold.co/15x15/66ffff/66ffff.png) `LoRaMiP GUI --> DevKit`

![#00ff00](https://placehold.co/15x15/00ff00/00ff00.png) `DevKit --> LoRaMiP GUI`


For more information regarding the entire list of commands, please refer to the command reference downloadable from the official [MIPOT](https://www.mipot.com) website, directly in the documents section of your product. Downloading is permitted upon registration to the site.

## Software needed for development
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- LoRaMiP GUI (downloadable from the official [MIPOT](https://www.mipot.com) website)

## Tools needed for development

- Programmer [STLINK](https://www.st.com/en/development-tools/st-link-v2.html)
- Serial Cable [TTL-232R-3V3](https://ftdichip.com/products/ttl-232r-3v3/)

## License

Shown in the LICENSE.md file

## Important Information

> [!CAUTION]
> This project is a firmware solution example on MIP Series.
> The main goal is to give a starting point on MIP Series Development.

> [!CAUTION]
> This project is provided "AS IS" with no warranties.

