# TP_Bus_Reseaux #
### Authors : JP Thomar, P. Boulot
### Professor : C. Barès 
### Topics : Lab Sessions on communiaction between STM32F4 and Raspberry Pi, sensors using I2C, CAN, UART, and WiFi 

##  Lab Session 1 : Communication between the pressure and temperature sensor BMP280 and the STM32F446 board using I²C. 

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
## Lab Session 2 : STM32 - Raspberry Pi interfacing

Download the image "Raspberry Pi OS (32-bit) Lite" on the SD card. Use Rpi_imager to install it on the SD card.
- Host name : raspberrypiJPP
- Id : TPBUSJPP , Login : TPBUSJPP
- Wifi configuration : SSID = ESE_Bus_Network / PW = **********
- Local settings : Time zone : Europe/Paris, Keyboard type : fr
