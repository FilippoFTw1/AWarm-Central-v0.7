
/*RCC*/
#define USART1_RCC_ENABLE					0x00004000
#define USART2_RCC_ENABLE					0x00020000
#define GPIOA_RCC_ENABLE					0x00000004
#define	GPIOB_RCC_ENABLE					0x00000008
#define AFIO_RCC_ENABLE						0x00000001
#define I2C1_RCC_ENABLE						0x00200000

/*CRC*/
#define WIDTH    							(8 * sizeof(unsigned short))
#define TOPBIT   							(1 << (WIDTH - 1))
#define POLYNOMIAL							0x1021

/*I2C*/
#define MUX_SLAVE_1 						0x70		//define address for the 1st i2c mux
#define MUX_SLAVE_2 						0x71		//define address for the 2nd i2c mux
#define APDS9930_ADDRESS 					0x39	//define address for the proximity sensor

/*I2C pins*/
#define SDA 								((uint16_t)0x0080)
#define SCL 								((uint16_t)0x0040)

/*APDS9930*/
#define APDS9930_ENABLE_REGISTER			0x00
#define APDS9930_ENABLE_COMMAND				0x00
#define APDS9930_PTIME_REGISTER				0x02
#define APDS9930_PTIME_COMMAND				0xFF
#define APDS9930_PPULSE_REGISTER			0x0E
#define APDS9930_PPULSE_COMMAND				0x08
#define APDS9930_CONTROL_REGISTER			0x0F
#define APDS9930_CONTROL_COMMAND			0x20
