
#include <main.h>
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_i2c.h>
#include <misc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//NEEDS TO BE DELETED
#define I2Cx I2C1

/*UART FROM ABox*/
unsigned char RX_buffer[200];												//Buffer for UART RX
unsigned int RX_buffer_counter;												//Counter for the buffer
u16 crc_check;																//Checksum
int crc_check_bytes = 0;													//Number of bytes expected
int received = 0;															//State of the IRQ handler
int status_buffer = 0;														//State of UART
int abox_communication_state = 1;
int abox_communication_select = 0;
unsigned long abox_timer;
unsigned int channel_select[8] = {0x0000, 0x1000, 0x2000, 0x3000, 0x4000, 0x5000, 0x6000, 0x7000};

/*UART TO RPi*/
unsigned char TX_buffer[200];
unsigned int TX_buffer_counter = 0;
unsigned char RX_rpi;
unsigned short buffer_done = 0;
unsigned short send = 0;
int rpi_communication_state = 2;
int rpi_communication_select = 2;
unsigned short rpi_abox_communication_select = 0;
unsigned short rpi_nTry = 0;

//Delay Funciton
char millis[8];
unsigned long msTicks;

/*CRC*/
unsigned short crcTable[256];

/*I2C*/
unsigned short i2c_nTry = 0;

/*ABox Informations*/
struct ABoxs{
	unsigned short status;
	unsigned char name[20];
	unsigned char surname[20];
	unsigned short battery;
	unsigned long long tag;
	unsigned long hwid;
	unsigned short A1;
	unsigned short A2;
	unsigned short T[100];
	signed short T_counter;
	unsigned short CO2[100];
	signed short CO2_counter;
	unsigned short CO2_last;
}  ABox[8];

/*Delay Methods*/
/*-------------------------------------------------------------*/
void Delay_Nus(uint32_t micros){
	micros *= (SystemCoreClock / 1000000) / 5;
	/* Wait till done */
	while (micros--);
}

void SysTick_Handler(){
	msTicks++;
}

void msInit(){
	msTicks = 0;
	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	// Configure the SysTick timer to overflow every 1 ms
	SysTick_Config(SystemCoreClock / 1000); //24bit tick counter
}

void simpleForDelay(int n){
	int i;
	for (i=0; i<n; i++);
}
/*-------------------------------------------------------------*/

/*Init Methods*/
/*-------------------------------------------------------------*/
/*RCC*/
void rcc_init(){
	RCC->APB2ENR |= USART1_RCC_ENABLE; 		//ENABLE USART1 CLOCK
	RCC->APB2ENR |= GPIOA_RCC_ENABLE;		//ENABLE GPIOA CLOCK
	RCC->APB2ENR |= GPIOB_RCC_ENABLE;		//ENABLE GPIOB CLOCK
	RCC->APB2ENR |= AFIO_RCC_ENABLE;		//ENABLE AFIO CLOCK

	RCC->APB1ENR |= USART2_RCC_ENABLE;		//ENABLE USART2 CLOCK
	RCC->APB1ENR |= I2C1_RCC_ENABLE;		//ENABLE I2C CLOCK
}

/*GPIO*/
void gpio_init(){
	GPIO_InitTypeDef GPIO_InitStruct;

	/*UART1 pins for ABox communication*/
	//RX
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//TX
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*UART2 pins for RPi communication*/
	//RX
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//TX
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Mux pins*/
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*I2C pins*/
	GPIO_InitStruct.GPIO_Pin = SDA | SCL;				//SCL on B6; SDA on B7
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;		//speed on the gpio 2MHz
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;		//we select alternative function and open drain
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*USART1*/
void uart_init(){
	USART_InitTypeDef UART_InitStruct;

	UART_InitStruct.USART_BaudRate = 57600;						//speed 57600baud
	UART_InitStruct.USART_WordLength = USART_WordLength_8b;		//data is 8 bits
	UART_InitStruct.USART_StopBits = USART_StopBits_1;			//with 1 stop bit
	UART_InitStruct.USART_Parity = USART_Parity_No;				//and no parity
	UART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//mode is in receive

	USART_Init(USART1, &UART_InitStruct);						//init the usart1
	USART_Init(USART2, &UART_InitStruct);

	USART_Cmd(USART1, ENABLE);									//enable usart1
	USART_Cmd(USART2, ENABLE);
}

/*Interrupt for the USART RX*/
void nvic_init(){
	NVIC_InitTypeDef NVIC_InitStruct;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStruct);
}

/*I2C Init*/
void i2c_init(){
	I2C_InitTypeDef I2C_InitStruct;

	I2C_Cmd(I2C1, ENABLE);
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;		//duty cycle of 50%
	I2C_InitStruct.I2C_ClockSpeed = 100000;				//speed for i2c 10kHz
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;				//address for the master
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;			//enable the ack
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	//choose 7bit address

	I2C_Init(I2Cx, &I2C_InitStruct);
}

/*Init CRC Table*/
void crc_init(void){
	unsigned short remainder;
	int	dividend;
	unsigned char bit;

    // Compute the remainder of each possible dividend.
    for (dividend = 0; dividend < 256; ++dividend){
        // Start with the dividend followed by zeros.
        remainder = dividend << (WIDTH - 8);
        // Perform modulo-2 division, a bit at a time.
        for (bit = 8; bit > 0; --bit){
            // Try to divide the current data bit.
            if (remainder & TOPBIT){
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else{
                remainder = (remainder << 1);
            }
        }
        // Store the result into the table.
        crcTable[dividend] = remainder;
    }
}

/*Interrupt*/
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	//check if there is data
	{
		//u8 newData = USART_ReceiveData(USART1);				//get the data
		u8 newData = (uint16_t)(USART1->DR & (uint16_t)0x01FF);
		if (received == 0 && newData == '{'){				//check if its a starting bit
			received = 1;
			RX_buffer[RX_buffer_counter++] = newData;		//put it in a buffer
		}
		else if (received == 1){
			RX_buffer[RX_buffer_counter++] = newData;
			if (newData == '}'){							//check if its a stop bit
				crc_check_bytes = 2;						//now we wait for the 2 bytes crc
				received = 2;
			}
		}
		else if (received == 2){
			crc_check = crc_check << 8 | newData;
			crc_check_bytes --;
			if (crc_check_bytes == 0){						//check if we got the 2 bytes crc
				received = 0;
				status_buffer = 1;
				USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);		//turn off the interrupt flag
			}
		}
	}
}
/*-------------------------------------------------------------*/

/*I2C Communication*/
/*Method to unstuck the I2C bus
 * 1st. We set the pins as Outputs
 * 2nd. We generate 10cycles, where the 9th acts like a NACK
 * 3nd. Then we generate a Stop Condition
 * 4th. We set the pins as I2C pins
 */
void unstuck(){
	unsigned int i;

	simpleForDelay(10000);

	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin = SDA;						//SCL on B6; SDA on B7
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;		//speed on the gpio 2MHz
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStruct);					//init the gpio structure

	GPIO_InitStruct.GPIO_Pin = SCL;						//SCL on B6; SDA on B7
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;		//speed on the gpio 2MHz
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStruct);					//init the gpio structure

	GPIOB->BSRR = SDA;									//set SDA high

	//Generate 10 cycles, where 9th act like NACK
	for (i = 0; i < 10; i++){
		GPIOB->BSRR = SCL;								//set SCL high
		simpleForDelay(2);

		GPIOB->BRR = SCL;								//set SCL low
		simpleForDelay(2);
	}

	//Generate a STOP condition
	GPIOB->BRR = SDA;									//set SDA low
	simpleForDelay(2);

	GPIOB->BSRR = SCL;									//set SCL high
	simpleForDelay(2);

	GPIOB->BSRR = SDA;									//set SDA high
	simpleForDelay(2);

	//Init the GPIO like I2C pins
	GPIO_InitStruct.GPIO_Pin = SDA | SCL;				//SCL on B6; SDA on B7
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;		//speed on the gpio 2MHz
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;		//we select alternative function and open drain
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

int select_port(u8 port){
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		unstuck();

	do{
		I2C_GenerateSTART(I2Cx, ENABLE);			//generate a start condition to take over the i2c bus
		simpleForDelay(1000);
	}while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB));		//check if a start bit is put on the i2c bus

	i2c_nTry = 0;
	do{
		if (i2c_nTry++ == 3)
			return -1;
		I2C_Send7bitAddress(I2Cx, MUX_SLAVE_1 << 1, I2C_Direction_Transmitter);		//send the address of the slave with a LSB - 0 for write
		simpleForDelay(1000);
	}while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));	//check if the transmiiter mode is selected

	i2c_nTry = 0;
	do{
		if (i2c_nTry++ == 3)
			return -1;
		I2C_SendData(I2Cx, 1 << port);						//send the port from the mux that u want to communicate with
		simpleForDelay(1000);
	}while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));			//check if the byte is sent

	do{
		I2C_GenerateSTOP(I2Cx, ENABLE);						//generate a stop condition
		simpleForDelay(1000);
	}while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));		//check if a stop condition is generated

	return 1;
}

void apds9930_write(u8 reg, u8 data){
	unsigned int i;

	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))		//checking if the i2c bus is free for use
		unstuck();

	do{
		I2C_GenerateSTART(I2Cx, ENABLE);							//generate a start condition to take over the i2c bus
		simpleForDelay(1000);
	}while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));		//check if a start bit is put on the i2c bus

	i2c_nTry = 0;
	do{
		if (i2c_nTry++ == 3)
			return -1;
		I2C_Send7bitAddress(I2Cx, APDS9930_ADDRESS << 1, I2C_Direction_Transmitter);	//send the address of the slave with a LSB - 0 for write
		simpleForDelay(1000);
	}while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));		//check if the transmiiter mode is selected

	i2c_nTry = 0;
	do{
		if (i2c_nTry++ == 3)
			return -1;
		I2C_SendData(I2Cx, 0x80 | reg);
		simpleForDelay(1000);
	}while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));				//check if the byte is sent, ACK from slave

	i2c_nTry = 0;
	do{
		if (i2c_nTry++ == 3)
			return -1;
		I2C_SendData(I2Cx, data);																//send the data that you want to write in the resiger
		simpleForDelay(1000);
	}while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));				//check if the byte is sent, ACK from slave

	do{
		I2C_GenerateSTOP(I2Cx, ENABLE);									//generate a stop condition
		simpleForDelay(1000);
	}while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));		//check if the condition is generated
}

void apds9930_init(){
	apds9930_write(APDS9930_ENABLE_REGISTER, APDS9930_ENABLE_COMMAND);				//write data in the ENABLE register
	apds9930_write(APDS9930_PTIME_REGISTER, APDS9930_PTIME_COMMAND);				//write data in the PTIME register
	apds9930_write(APDS9930_PPULSE_REGISTER, APDS9930_PPULSE_COMMAND);				//write data in the PPULSE register
	apds9930_write(APDS9930_CONTROL_REGISTER, APDS9930_CONTROL_COMMAND);			//write data in the CONTROL register
}

u16 apds9930_read(){
	u16 dataL, dataH = 0;

	//I2C_AcknowledgeConfig(I2Cx, ENABLE);				//enable the ACK bit
	I2C1->CR1 |= 0x0400;

	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))		//checking if the i2c bus is free for use
		unstuck();

	do{
		//I2C_GenerateSTART(I2Cx, ENABLE);				//generate a start condition to take over the i2c bus
		I2C1->CR1 |= 0x0100;
		simpleForDelay(1000);
	}while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB));		//check if a start bit is put on the i2c bus

	i2c_nTry = 0;
	do{
		if (i2c_nTry++ == 3)
			return -1;
		I2C1->DR = (APDS9930_ADDRESS << 1) & 0xFFFE;
		//I2C_Send7bitAddress(I2Cx, APDS9930_ADDRESS << 1 , I2C_Direction_Transmitter);	//send the slave APDS9930 address
		simpleForDelay(1000);
	}while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));		//check if transmitter mode is selected, ACK from slave

	i2c_nTry = 0;
	do{
		if (i2c_nTry++ == 3)
			return -1;
		//I2C_SendData(I2Cx, 0xA0 | 0x18);
		I2C1->DR = 0xA0 | 0x18;			//send data from what register you want to read, in this case is 0x18 - DATAL
		simpleForDelay(1000);
	}while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE));		//test on TXE flag

	do{
		//I2C_GenerateSTART(I2Cx, ENABLE);					//generate ReStart condition
		I2C1->CR1 |= 0x0100;
		simpleForDelay(1000);
	}while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB));		//check if a start condition is generated

	do{
		if (i2c_nTry++ == 3)
			return -1;
		//I2C_Send7bitAddress(I2Cx, APDS9930_ADDRESS << 1, I2C_Direction_Receiver);		//send the slave address with LSB - 1, you want to read
		I2C1->DR = (APDS9930_ADDRESS << 1) | 0x0001;
		simpleForDelay(1000);
	}while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));			//check if receiver mode is selected on master


	I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);	//indicates that the next byte will be the last received byte.
	I2C_AcknowledgeConfig(I2Cx, DISABLE);					//disable the ACK bit

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));	//check if a byte is received
	dataL = I2C_ReceiveData(I2Cx);									//read the DATAL register

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));	//check if a byte is received
	dataH = I2C_ReceiveData(I2Cx);									//read the DATAH register

	do{
		I2C_GenerateSTOP(I2Cx, ENABLE);									//genereate a stop condition
		simpleForDelay(1000);
	}while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF));					//check if the condition is generated

	return ((dataH<<8)|dataL);										//return the data as 16bit
}
/*-------------------------------------------------------------*/

/*ABox Communication*/
/*-------------------------------------------------------------*/
unsigned short crcFast(unsigned char const message[], int nBytes){
	unsigned short remainder = 0xFFFF;
    unsigned char data;
	int byte;

    // Divide the message by the polynomial, a byte at a time.
    for (byte = 0; byte < nBytes; ++byte)
    {
        data = message[byte] ^ (remainder >> (WIDTH - 8));
  		remainder = crcTable[data] ^ (remainder << 8);
    }
    // The final remainder is the CRC.
    return remainder;
}

unsigned long long pow10(unsigned short size){
	unsigned long long result = 1;
	unsigned short i = 0;

	for (i = 0; i < size; i++){
		result = result * 10;
	}

	return result;
}

unsigned long long correct(unsigned char *data, unsigned short nByte){
	unsigned long long result = 0;
	unsigned short i = 0;

	for (i = 0; i < nByte; i++){
		result += pow10(i)*(data[nByte - i - 1] - '0');
	}

	return result;
}

int json(struct ABoxs *abox){
	unsigned char var[20] = {0};
	unsigned char var1[2] = {0};
	unsigned int i = 0, j = 0;

	for (i = 0; i < RX_buffer_counter; i++){
		switch (RX_buffer[i]){
		case '"': case '{': case '[':
			break;
		case ':':
			strcpy(var1, var);
			memset(var, 0, 20);
			j = 0;
			break;
		case ',': case '}': case ']':
			switch (var1[0]){
			case 'N':
				strcpy(abox->name, var);
				break;
			case 'S':
				strcpy(abox->surname, var);
				break;
			case 'B':
				abox->battery = correct(var, 4);
				break;
			case 'H':
				abox->hwid = correct(var, 8);
				break;
			case 'I':
				abox->tag = correct(var, 13);
				break;
			case 'A':
				if (var1[1] == '1')
					abox->A1 = var[0]-'0';
				else if (var1[1] == '2')
					abox->A2 = var[0]-'0';
				break;
			case 'T':
				abox->T[abox->T_counter++] = correct(var, 4);
				break;
			case 'C':
				abox->CO2[abox->CO2_counter++] = correct(var, 2);
				abox->CO2_last = correct(var, 2);
				break;
			}
			memset(var, 0, 20);
			j = 0;
			if (RX_buffer[i] == ']')
				i++;
			if (RX_buffer[i] == '$')
				i++;
			break;
		default:
			var[j++] = RX_buffer[i];
			break;
		}
	}
	return 1;
}

int goto_next_ABox(unsigned short ABox){
	ABox++;
	if (ABox >= 8)
		ABox = 0;

	return ABox;
}

void abox_reset_values(struct ABoxs *abox){
	abox->status = 0;
	memset(abox->name, 0, 20);
	memset(abox->surname, 0, 20);
	abox->battery = 0;
	abox->tag = 0;
	abox->hwid = 0;
	abox->A1 = 0;
	abox->A2 = 0;
	memset(abox->T, 0, 100);
	abox->T_counter = 0;
	memset(abox->CO2, 0, 100);
	abox->CO2_counter = 0;
	abox->CO2_last = 0;
}

void abox_communication(){
	int generated_CRC;
	u16 data_PS1;
	unsigned int i;
	unsigned int i2c_check;

	/*STATE 1*/
	if (abox_communication_state == 1){
		//check the proximity sensor if there is box
	    i2c_check = select_port(0);							//select port 0 on the mux
    	for(i=0;i<2;i ++);
    	apds9930_write(APDS9930_ENABLE_REGISTER, 0x05);				//power up the slave APDS9930
    	for(i=0;i<2;i++);

    	data_PS1 = apds9930_read();							//read the data from the proximity sensors
    	for(i=0;i<10;i++);
    	if (data_PS1 >= 800){
    		//check if the abox status is 0
    		if(ABox[abox_communication_select].status == 0){
    			//select the channel
    //			GPIO_SetBits(GPIOB, channel_select[abox_communication_select]);
    //			GPIO_ResetBits(GPIOB, 0x7000 ^ channel_select[abox_communication_select]);
    			//send request to the ABox
    			USART_SendData(USART1, 'R');
    			for(i=0;i<100;i++);
    			USART_SendData(USART1, abox_communication_select + '0');
    			goto STATE_2;
    		}
    		else
    			abox_communication_select = goto_next_ABox(abox_communication_select);
    	}
    	else
    		//Go next ABox
    		abox_communication_select = goto_next_ABox(abox_communication_select);
	}

	/*STATE 2*/
	else if (abox_communication_state == 2){
		//enable flag interrupt
STATE_2:
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		abox_timer = msTicks;
		abox_communication_state = 3;
	}

	/*STATE 3*/
	else if (abox_communication_state == 3){
		//check if we received the buffer
		if (status_buffer)
			abox_communication_state = 4;
		//check if the timer is over 1s
//		else if (msTicks - abox_timer > 1000){
//			abox_communication_select = goto_next_ABox(abox_communication_select);
//			abox_communication_state = 1;
//		}
	}

	/*STATE 4*/
	else if (abox_communication_state == 4){
		//generate CRC
		generated_CRC = crcFast(RX_buffer, RX_buffer_counter);
		//check CRC
		if (generated_CRC == crc_check){
			ABox[abox_communication_select].status = 1;
			//Generate JSON
			json(&ABox[abox_communication_select]);
			//ACK
			USART_SendData(USART1, 'A');
			for(i=0;i<100;i++);
			//Check if there is '$', that repesent that we got the last package from the ABox and need to go to the next ABox
			if (RX_buffer[RX_buffer_counter-2] != '$'){
				abox_communication_state = 2;
			}
			else {
				//END COMMUNICATIN
				abox_communication_state = 1;
				ABox[abox_communication_select].status = 2;
				//Next ABox
				abox_communication_select = goto_next_ABox(abox_communication_select);
			}
		}
		else {
			//NACK
			USART_SendData(USART1, 'N');
			for(i=0;i<100;i++);
			abox_communication_state = 2;
		}
		status_buffer = 0;
		RX_buffer_counter = 0;
	}
}
/*-------------------------------------------------------------*/

/*RPi Communication*/
/*-------------------------------------------------------------*/
void append(unsigned char* data){
	while (*data)
		TX_buffer[TX_buffer_counter++] = *data++;
}

unsigned char* decombined(unsigned long long data, unsigned short nByte){
	unsigned char result[15] = {'0'};
	unsigned short i = 0;

	for (i = 0; i < nByte; i++){
		unsigned int num = ((data/pow10(i))%10);
		result[nByte-i-1] = ((data/pow10(i))%10) + '0';
	}

	return result;
}

void send_char(char c){
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE)==RESET);
	USART_SendData(USART2, c);
}

void send_string(const char* s){
	while(*s)
		send_char(*s++);
}

void add(unsigned char data){
	TX_buffer[TX_buffer_counter++] = data;
}

void fill_buffer(char buffer_type, struct ABoxs *abox){
	unsigned char buffer[15] = {0};
	unsigned char var[2];
	unsigned int i, j;

	if (buffer_type == 'G')
		strcpy(buffer, "PHINSBA1A2L");
	else
		strcpy(buffer, "PTC");

	//empty the buffer
	memset(TX_buffer, 0, 200);
	TX_buffer_counter = 0;

	add('{');
	for(i=0;i<strlen(buffer);i++){
		add('"');
		add(buffer[i]);
		if (buffer[i] == 'A')
			add(buffer[i+1]);
		add('"');
		add(':');

		switch (buffer[i]){
		case 'P': case 'H': case 'I': case 'N': case 'S': case 'A': case 'L':
			var[0] = '"';
			var[1] = '"';
			break;
		case 'T': case 'C':
			var[0] = '[';
			var[1] = ']';
		default:
			break;
		}

		add(var[0]);

		switch (buffer[i]){
		case 'P':
			add((char)(rpi_abox_communication_select + '0'));
			break;
		case 'H':
			append(decombined(abox->hwid, 8));
			break;
		case 'I':
			append(decombined(abox->tag, 13));
			break;
		case 'N':
			append(abox->name);
			break;
		case 'S':
			append(abox->surname);
			break;
		case 'B':
			append(decombined(abox->battery, 4));
			break;
		case 'A':
			if (buffer[i+1] == '1')
				add((char)(abox->A1+'0'));
			else
				add((char)(abox->A2+'0'));
			break;
		case 'L':
			append(decombined(abox->T_counter, 1));
			break;
		case 'T':
			for(j=0;j<21;j++){
				append(decombined(abox->T[abox->T_counter-1], 4));
				abox->T_counter--;
				if (abox->T_counter == 0)
					break;
				else if(j+1 != 21)
					append(",");
			}
			break;
		case 'C':
			for(j=0;j<21;j++){
				append(decombined(abox->CO2[abox->CO2_counter-1], 2));
				abox->CO2_counter--;
				if (abox->CO2_counter == 0)
					break;
				else if (i+1 != 21)
					append(",");
			}
			break;
		default:
			break;
		}

		add(var[1]);
		if (buffer[i] != 'L' && buffer[i] != 'C')
			add(',');

		if (buffer[i] == 'A')
			i++;
	}

	add('}');
}

void rpi_communication(){
	unsigned short i = 0;
	unsigned short test;
	u8 a[2];

	/*STATE 1*/
	/*
	 * Wait for the RPi to join Serial Communication
	*/
	if (rpi_communication_state == 1){
		if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE)){
			RX_rpi = USART_ReceiveData(USART2);
			if (RX_rpi == 'C'){
				rpi_communication_select = 1;
				rpi_communication_state = 2;
			}
		}
	}

	/*STATE 2*/
	/*
	 * Check if status of the ABox is 2 (it's ready to be send to the RPi)
	 * if its not go to the next ABox
	 */
	else if (rpi_communication_state == 2){
		if (ABox[rpi_abox_communication_select].status == 2){
			ABox[rpi_abox_communication_select].status = 3;
			rpi_communication_state = 3;
			rpi_communication_select = 1;
		}
		else
			rpi_abox_communication_select = goto_next_ABox(rpi_abox_communication_select);
	}

	/*STATE 3*/
	/*
	 * Fill the buffer and send it
	 * rpi_communication_select = 1, Fill the buffer with General Info
	 * rpi_communication_select = 2, Fill the buffer with info for temp and CO2
	 */
	else if (rpi_communication_state == 3){
		if (rpi_communication_select == 1)
			fill_buffer('G', &ABox[rpi_abox_communication_select]);
		else if (rpi_communication_select == 2)
			fill_buffer('B', &ABox[rpi_abox_communication_select]);

		test = crcFast(TX_buffer, TX_buffer_counter);
		a[0] = (test & 0xFF00) >> 8;
		a[1] = test & 0x00FF;
		if(ABox[rpi_abox_communication_select].T_counter == 0)
			add('$');
		else
			add(' ');
		append(a);
		send_string(TX_buffer);
		rpi_nTry = 0;
		rpi_communication_state = 4;
	}

	/*STATE 4*/
	/*
	 * Wait for response from the RPi
	 * if its ACK, check what state are you, if you are finished with transfering data to RPi go to the next ABox
	 * if its NACK, resends the buffer
	 */
	else if (rpi_communication_state == 4){
		//Wait for ACK or NACK
		if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE)){
			RX_rpi = USART_ReceiveData(USART2);
			if (RX_rpi == 'A'){
				if (rpi_communication_select == 1){
					rpi_communication_select = 2;
					rpi_communication_state = 3;
				}
				else if (rpi_communication_select == 2){
					if (TX_buffer[TX_buffer_counter-3] == '$'){
						rpi_communication_select = 1;
						rpi_communication_state = 2;
						//clear the abox
						abox_reset_values(&ABox[rpi_abox_communication_select]);
						//Go to next box
						rpi_abox_communication_select = goto_next_ABox(rpi_abox_communication_select);
					}
					else{
						rpi_communication_select = 2;
						rpi_communication_state = 3;
					}
				}
			}
			//REsend the buffer
			else if (RX_rpi == 'N'){
				if (rpi_nTry++ == 1){
					rpi_communication_state = 2;
				}
				else
					send_string(TX_buffer);
			}

		}
	}
}
/*-------------------------------------------------------------*/

int main(void){
	unsigned int i;
	unsigned int i2c_check;
	unsigned long msStart = 0;

	/*Init all structure*/
	rcc_init();
	gpio_init();
	uart_init();
	nvic_init();
	i2c_init();
	crc_init();
	msInit();

	//Setup the APDS9930
	i2c_check = select_port(0);				//select port 0 on the mux
	simpleForDelay(2);
	apds9930_init();			//init the registers on the selected APDS9930
	simpleForDelay(10);

	i2c_check = select_port(6);				//select port 6 on the mux
	simpleForDelay(2);
	apds9930_init();			//init the registers on the selected APDS9930
	simpleForDelay(10);

	//enable S2,S1,S0
	GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);

	//Reset the values of every ABox
	for (i = 0; i < 8; i++)
		abox_reset_values(&ABox[i]);

    while(1){
    	//ABox Communication
	    abox_communication();
	    //RPi communication
	    rpi_communication();

	    //Code that will be execute every 1s
//    	if(msTicks - msStart > 1000){
//    	    msStart = msTicks;
//    	    rpi_communication();
//    	}
    }
    return 0;
}
