# TP_Bus_Reseaux #
### Authors : JP Thomar, P. Boulot
### Professor : C. Barès 
### Topics : Lab Sessions on communiaction between STM32F4 and Raspberry Pi, sensors using I2C, CAN, UART, and WiFi 

##  Lab Session 1 : I2C Bus.  

Objective: Interfacing a STM32 with I2C sensors  
The first step is to set up communication between the microcontroller and sensors (temperature, pressure, accelerometer...) via the I2C bus.
The sensor has two I2C components, which share the same bus. The STM32 will act as master on the bus.
The STM32 code will be written in C language, using the HAL library.

```C
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
```
## Lab Session 2 : STM32 - Raspberry Pi 0 WIFI interfacing
During this session we are going to establish the communication between the two boards Raspberry Pi 0 WIFI ("RPi" below) and STM32.

### Setting the Raspberry Pi 0 Wifi
First, we follow a few steps to prepare the RPi: 

- Download the image "Raspberry Pi OS (32-bit) Lite" on the SD card. 
- Use the Rpi_imager software to install it on the SD card.  
Host name : raspberrypiJPP  
Id : TPBUSJPP , Login : TPBUSJPP  
Wifi configuration : SSID = ESE_Bus_Network / PW = **********  
Local settings : Time zone : Europe/Paris, Keyboard type : fr  

- A free IP adress in the network is assigned to the RPi : 192.168.88.231
- Connect the Pi0 to the PC via SSH using the following command : "ssh TPBUSJPP@192.168.88.231".

### Serial port  
Loopback  
We pluged the serial port of the RPi in such way that Rx and TX are loopped, and used Minicom to test it and verify the access to the loopback. 

Communication with the STM32  
To communicate with the STM32, an independant UART port is used (UART 4, cf Lab session 1).  
We modify the printf fonction to enable it to display on both serial port (UART 2 and UART 4).  

```C
//redefinition of printf

```
