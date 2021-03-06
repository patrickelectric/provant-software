/**
  ******************************************************************************
  * @file    modules/common/c_common_i2c.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-Dezember-2013
  * @brief   Implmentacão das funções de I2C.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_common_i2c.h"

/** @addtogroup Common_Components
  * @{
  */

/** @addtogroup Common_Components_I2C
  *  \brief Funções para uso da I2C (I2C1).
  *
  *  Posto que apenas a I2C1 será usada, as funções terão esta "hard-coded", via define:
  *
  *  \code{.c}
  *  #define I2Cx I2C1 //tipo I2C_TypeDef*
  *  \endcode
  *
  *  Fonte da biblioteca, <a href="https://github.com/Torrentula/STM32F4-examples/blob/master/I2C%20Master/main.c">aqui</a>. Exemplo de utilização:
  *  \code{.c}
  *  #define SLAVE_ADDRESS 0x3D // the slave address (example)
  *
  *	 int main(void){
  *
  *        init_I2C1(); // initialize I2C peripheral
  *
  *        uint8_t received_data[2];
  *
  *        while(1){
  *
  *                c_common_i2c_start(SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
  *                																	// shifts address for 7 bit mode (makes room for R/W bit)
  *                c_common_i2c_write(0x20); // write one byte to the slave
  *                c_common_i2c_write(0x03); // write another byte to the slave
  *                c_common_i2c_stop(); // stop the transmission
  *
  *                c_common_i2c_start(SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
  *                received_data[0] = I2C_read_ack(I2C1); // read one byte and request another byte
  *                received_data[1] = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
  *       }
  *	 }
  *	 \endcode
  *  @{
  */

/* Private typedef -----------------------------------------------------------*/
#define I2Cx 	I2C1
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializa a I2C1 em PB8 e PB9 (SCL e SDA).
 *
 */
void c_common_i2c_init(){

        GPIO_InitTypeDef GPIO_InitStruct;
        I2C_InitTypeDef I2C_InitStruct;

        // enable APB1 peripheral clock for I2C1
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        // enable clock for SCL and SDA pins
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

        /* setup SCL and SDA pins
         * You can connect I2C1 to two different
         * pairs of pins:
         * 1. SCL on PB6 and SDA on PB7
         * 2. SCL on PB8 and SDA on PB9 <-----------
         */
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; // set output to open drain --> the line has to be only pulled low, not driven high
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;   // enable pull up resistors
        GPIO_Init(GPIOB, &GPIO_InitStruct);         // init GPIOB

        // Connect I2C1 pins to AF
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); // SCL
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // SDA

        // configure I2C1
        I2C_InitStruct.I2C_ClockSpeed = 100000; // 100kHz
        I2C_InitStruct.I2C_Mode = I2C_Mode_I2C; // I2C mode
        I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 50% duty cycle --> standard
        I2C_InitStruct.I2C_OwnAddress1 = 0x00;  		// own address, not relevant in master mode
        I2C_InitStruct.I2C_Ack = I2C_Ack_Disable; 		// disable acknowledge when reading (can be changed later on)
        I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
        I2C_Init(I2C1, &I2C_InitStruct); 	    // init I2C1

        // enable I2C1
        I2C_Cmd(I2C1, ENABLE);
}

/** \brief Emite uma condição de início de transmissão e envia o endereço do escravo com o bit de R/W.
 *
 * @param address 	Endereço de 7 bits do escravo.
 * @param direction	Direção da transmissão. Pode ser:
 * 						\em I2C_Direction_Transmitter \em para <b> Master transmitter mode </b>, ou
 * 						\em I2C_Direction_Receiver \em para <b> Master receiver mode</b>.
 */
void c_common_i2c_start(/*I2C_TypeDef* I2Cx,*/ uint8_t address, uint8_t direction) {
        // wait until I2C1 is not busy anymore
        while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

        // Send I2C1 START condition
        I2C_GenerateSTART(I2Cx, ENABLE);

        // wait for I2C1 EV5 --> Slave has acknowledged start condition
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

        // Send slave Address for write
        I2C_Send7bitAddress(I2Cx, address, direction);

        /* wait for I2C1 EV6, check if
         * either Slave has acknowledged Master transmitter or
         * Master receiver mode, depending on the transmission
         * direction
         */
        if(direction == I2C_Direction_Transmitter){
                while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
        }
        else if(direction == I2C_Direction_Receiver){
                while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
        }
}

/** \brief Envia um byte ao escravo.
 *
 * @param data Byte a ser enviado.
 */
void c_common_i2c_write(/*I2C_TypeDef* I2Cx,*/ uint8_t data) {
        I2C_SendData(I2Cx, data);
        // wait for I2C1 EV8_2 --> byte has been transmitted
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/** \brief Lê um byte do escravo e confima (acknowledges) o byte (requisita um próximo).
 *
 * @retval Byte lido.
 */
uint8_t c_common_i2c_readAck(/*I2C_TypeDef* I2Cx,*/) {
        // enable acknowledge of recieved data
        I2C_AcknowledgeConfig(I2Cx, ENABLE);
        // wait until one byte has been received
        while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
        // read data from I2C data register and return data byte
        uint8_t data = I2C_ReceiveData(I2Cx);
        return data;
}

/** \brief Lê um byte do escravo mas não confima (doesn't acknowledges) o byte.
 *
 * @retval Byte lido.
 */
uint8_t c_common_i2c_readNack(/*I2C_TypeDef* I2Cx,*/) {
        // disabe acknowledge of received data
        // nack also generates stop condition after last byte received
        // see reference manual for more info
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
        I2C_GenerateSTOP(I2Cx, ENABLE);
        // wait until one byte has been received
        while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
        // read data from I2C data register and return data byte
        uint8_t data = I2C_ReceiveData(I2Cx);
        return data;
}

/** \brief Emite uma condição de parada e libera o barramento.
 *
 */
void c_common_i2c_stop(/*I2C_TypeDef* I2Cx*/) {
        // Send I2C1 STOP Condition
        I2C_GenerateSTOP(I2Cx, ENABLE);
}

/** \brief Lê uma quantidade de bytes de um dado endereço em um determinado dispositivo.
 *
 *	Exemplo: lê 1 byte a partir do endereço de memória 0x00 do dispositivo com endereço
 *	0x68 no barramento, e salva o resultado em ITG3205_ID.
 *	\code{.c}
 *	c_common_i2c_readBytes(0x68, 0x00, 1, &ITG3205_ID);
 *	\endcode
 *
 *	@param device Endereço do dispositivo no barramento.
 *	@param address Endereço de memória a ser lido (comando antes da leitura).
 *	@param bytesToRead Quantos bytes são esperados.
 *	@param recvBuffer Ponteiro para um buffer com tamanho mínimo de \b bytesToRead, no qual a resposta será armazenada.
 *
 */
void c_common_i2c_readBytes(uint8_t device, uint8_t address, char bytesToRead, uint8_t * recvBuffer) {
	c_common_i2c_start(device<<1, I2C_Direction_Transmitter);
	c_common_i2c_write(address);
	c_common_i2c_stop();

	c_common_i2c_start(device<<1, I2C_Direction_Receiver);
	for(int i=0; i<bytesToRead-1; i++)
		recvBuffer[i] = c_common_i2c_readAck();

	recvBuffer[bytesToRead-1] = c_common_i2c_readNack();
	c_common_i2c_stop();
}

/** \brief Escreve um byte num dispositivo com um dado endereço.
 *
 * @param device Endereço do dispositivo no barramento.
 * @param address Endereço a ser escrito no dispositivo.
 * @param byteToWrite Byte a ser escrito.
 */
void c_common_i2c_writeByte(uint8_t device, uint8_t address, uint8_t byteToWrite) {
	c_common_i2c_start(device<<1, I2C_Direction_Transmitter);
	c_common_i2c_write(address);
	c_common_i2c_write(byteToWrite);
	c_common_i2c_stop();
}

/* IRQ handlers ------------------------------------------------------------- */


/**
  * @}
  */

/**
  * @}
  */

