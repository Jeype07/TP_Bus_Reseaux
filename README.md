# TP_Bus_Reseaux #
### Authors : JP Thomar, P. Boulot
### Professor : C. Barès 
### Topics : Lab Sessions on communiaction between STM32F4 and Raspberry Pi0, and sensors using I2C, CAN, UART, and WiFi 

##  Lab Session 1 : I2C Bus.  

Objective: Interfacing a STM32 with I2C sensors  

The first step is to set up communication between the microcontroller and sensors (temperature, pressure, accelerometer...) via the I2C bus.
The sensor has two I2C components, which share the same bus. The STM32 will act as master on the bus.
The STM32 code will be written in C language, using the HAL library.

### 2.1 BMP280 Sensor

From the datasheet of the pressure sensor BMP280 we can determine the following pieces of information : 
- possible I2C adresses for this component : 0x76 and 0x77
- the register and the value to identify this component : register : 0xD0 "id", value : 0x58
- the register and value to set the component in normal mode : [0:1] bits in control register 0xF4 have to be set to 11
- the registers containing the component calibration : registers "calib25" to "calib00" with adresses 0xA1 to 0x88
- the temperature records (and format) : "temp" registers contains the rax temperature measurement data output ut[19:0] at the adresses 0xFA, 0xFB, and 0xFC 
- the pressure registers (and format) : "press" registers contains the raw pressure measurement data output up[19:0] at the adresses 0xF7, 0xF8, and 0xF9
- the functions for calculating the temperature and pressure compensated, in 32-bit integer format : cf datasheet p.23

### 2.2 STM32 Setup

For this lab sessions, we will use the STM32446RETX board on STM32CubeIDE with the following connections : 
- PB8 : I2C1_SDA
- PB9 : I2C1_SCL
- PA2 : USART2_TX (USB)
- PA3 : USART2_RX
- PA0 : UART4_TX (communication with raspberry pi)
- PA1 : UART4_RX
- PA12 : CAN1_TX
- PA11 : CAN1_RX

We modify now the printf fonction to so that it returns its strings on the UART to USB link, by adding the following code to the stm32f4xx_hal_msp.c file :  

```C
/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart2;
/* USER CODE END PV */
/* USER CODE BEGIN Macro */
#ifdef __GNUC__ /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf    set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END Macro */
/* USER CODE BEGIN 1 */
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
/* USER CODE END 1 */
```

Echo type test of the UART compilation chain and communication over USB :  
in main loop : 
```C

uint8_t data; 
while (1)
	{
		//code bloquant pour écho
		HAL_UART_Receive( &huart2, &data, 1, HAL_MAX_DELAY );
		printf("\r\n");
		printf("%s\r\n",&data);
		HAL_UART_Transmit( &huart2, &data, 1, HAL_MAX_DELAY );
	}
```
### I2C communication 
#### BMP280 Identification 
Reading a register's data using I2C is as follows :  
1 - send the address of the registry ID  
2 - receive 1 byte corresponding to the contents of the register  
```C
* Private define ------------------------------------------------------------*/
#define BMP_ADDR 0x77<<1 // BMP280 address
#define BMP_ID_REG 0xD0 // adress of the ID register
```

in the main loop : 
```C
uint8_t id_buf[1];

//question réponse capteur avec I2C pour ID capteur
id_buf[0]= BMP_ID_REG;
HAL_I2C_Master_Transmit(&hi2c1,BMP_ADDR,id_buf,1,HAL_MAX_DELAY);
HAL_I2C_Master_Receive(&hi2c1,BMP_ADDR,id_buf,1,HAL_MAX_DELAY);
printf("BMP280 ID : %x\r\n",id_buf[0]);
```

#### Communication with BMP280
In I2C, the writing in a register is made as follows:  
1 - Send the address of the register to be written, followed by the value of the register  
2 - If received immediately, value received will be the new register value  

We will use the following configuration: normal mode, oversampling pressure x16, temperature oversampling x2. We will thus modify the 0xF4 "ctrl_meas" register with the following value :  
01010111 (010 oversampling t x2, 101 oversampling p x16,  11 mode normal)

```C
* Private define ------------------------------------------------------------*/
#define BMP_ADDR_MODE 0xF4 // address of the "ctrl_meas" reg to set the modes/config
#define BMP_MODE 01010111 //  010 oversampling t x2, 101 oversampling p x16,  11 mode normal
```
main loop : 
```C
//Configuration et vérification du capteur
uint8_t data_config[2];
data_config[0]= BMP_ADDR_MODE;
data_config[1]= BMP_MODE;
HAL_I2C_Master_Transmit(&hi2c1,BMP_ADDR,buf,1,HAL_MAX_DELAY);
HAL_I2C_Master_Receive(&hi2c1,BMP_ADDR,buf,1,HAL_MAX_DELAY);
printf("Registre : %x\r\n",buf[0]);
printf("Mode : %x\r\n",buf[1]);
```
#### Calibration, temperature and pressure recovery

We will now retrieve the contents of the registers containing the BMP280 calibration in one go.
They are the followings ones : "calib25" to "calib00" with adresses 0xA1 to 0x88
```C
typedef struct { // structure containing calibration registers names (datasheet p.21)
    uint16_t dig_T1;  //0x88/0x89
    int16_t dig_T2;   //0x8A/0x8B
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;  //0x9C/0x9D
    int16_t dig_P9;  //0x9E/0x9F
} Struct_CalibDataNames;
```
In the main loop :  
```C
// Retrieving of calibration Data
calib_reg[0] = BMP_CALIB_REG;
HAL_I2C_Master_Transmit(&hi2c1, BMP_ADDR, calib_reg, 1, HAL_MAX_DELAY);
HAL_I2C_Master_Receive(&hi2c1, BMP_ADDR, calib_data, BMP_CALIB_DATA_LENGTH, HAL_MAX_DELAY);
calib_names->dig_T1 = (uint16_t)((calib_data[1] << 8) | calib_data[0]);
calib_names->dig_T2 = (int16_t)((calib_data[3] << 8) | calib_data[2]);
calib_names->dig_T3 = (int16_t)((calib_data[5] << 8) | calib_data[4]);
calib_names->dig_P1 = (uint16_t)((calib_data[7] << 8) | calib_data[6]);
calib_names->dig_P2 = (int16_t)((calib_data[9] << 8) | calib_data[8]);
calib_names->dig_P3 = (int16_t)((calib_data[11] << 8) | calib_data[10]);
calib_names->dig_P4 = (int16_t)((calib_data[13] << 8) | calib_data[12]);
calib_names->dig_P5 = (int16_t)((calib_data[15] << 8) | calib_data[14]);
calib_names->dig_P6 = (int16_t)((calib_data[17] << 8) | calib_data[16]);
calib_names->dig_P7 = (int16_t)((calib_data[19] << 8) | calib_data[18]);
calib_names->dig_P8 = (int16_t)((calib_data[21] << 8) | calib_data[20]);
calib_names->dig_P9 = (int16_t)((calib_data[23] << 8) | calib_data[22]);

```

In the infinite loop of the STM32, we retrieve the raw values of temperature and pressure, then we send to the serial port the 32 bit uncompensated values of the temperature pressure.
```C
#define BMP_TEMP_PRESS_REG 0xF7 // 1st press register address
#define BMP_TEMP_PRESS_DATA_LENGTH 6 // size of press + temp registers
```

```C
while (1)
{
	//Retrieving the raw temp and press values
	uint8_t raw_data[BMP_TEMP_PRESS_DATA_LENGTH];
	uint8_t reg = BMP_TEMP_PRESS_REG;
	HAL_I2C_Master_Transmit(&hi2c1, BMP_ADDR, &reg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, BMP_ADDR, raw_data, BMP_TEMP_PRESS_DATA_LENGTH, HAL_MAX_DELAY);
	*press = (int32_t)(((raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]) >> 4);
	*temp = (int32_t)(((raw_data[3] << 16) | (raw_data[4] << 8) | raw_data[5]) >> 4);
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

## Lab Session 3 : REST interface
Id : jpp , Login : TPBUSJPP  
Wifi configuration : SSID = ESE_Bus_Network / PW = **********  
Local settings : Time zone : Europe/Paris, Keyboard type : fr  

- A free IP adress in the network is assigned to the RPi : 192.168.88.231
- Connect the Pi0 to the PC via SSH using the following command : "ssh jpp@192.168.88.231".

```py
from flask import Flask
app = Flask(__name__)

@app.route('/')
def hello_world():
    return 'Hello, World!\n'
```
image page 1

```py
from flask import Flask
app = Flask(__name__)

@app.route('/')
def hello_world():
    return 'Hello, World!\n'

welcome = "Welcome to 3ESE API!"

@app.route('/api/welcome/')
def api_welcome():
    return welcome

@app.route('/api/welcome/<int:index>')
def api_welcome_index(index):
    return welcome[index]
```

image page 2

@app.route role : add elements to the url to run corresponding function, here to print welcome sting

<int:index> role : add element to the url to run corresponding function, here print element at index position of welcome string

1st solution
```py
import Flask
from flask import jsonify

app = Flask(__name__)

@app.route('/')
def hello_world():
    return 'Hello, World!\n'

welcome = "Welcome to 3ESE API!"

@app.route('/api/welcome/')
def api_welcome():
    return welcome

@app.route('/api/welcome/<int:index>')
def api_welcome_index(index):
    return json.dumps({"index": index, "val": welcome[index]})
```
image html

image json

```py
import Flask
from flask import jsonify

app = Flask(__name__)

@app.route('/')
def hello_world():
    return 'Hello, World!\n'

welcome = "Welcome to 3ESE API!"

@app.route('/api/welcome/')
def api_welcome():
    return welcome

@app.route('/api/welcome/<int:index>')
def api_welcome_index(index):
    return jsonify({"index": index, "val": welcome[index]}), {"Content-Type": "application/json"}
```

image json 2

error 404

```py
from flask import Flask
from flask import jsonify
from flask import render_template
from flask import abort
app = Flask(__name__)

@app.route('/')
def hello_world():
    return 'Hello, World!\n'

welcome = "Welcome to 3ESE API!"

@app.route('/api/welcome/')
def api_welcome():
    return welcome

@app.route('/api/welcome/<int:index>')
def api_welcome_index(index):
    if index<0 or index>len(welcome):
        abort(404)
    else:
        return jsonify({"index": index, "val": welcome[index]}), {"Content-Type": "application/json"}

@app.errorhandler(404)
def page_not_found(error):
    return render_template('page_not_found.html'), 404
```
image error 404

 
