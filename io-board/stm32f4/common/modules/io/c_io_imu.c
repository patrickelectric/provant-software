/**
  ******************************************************************************
  * @file    modules/io/c_io_imu.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   Funções para IMUs (incialmente baseadas no CIs ITG3205 e ADXL345).
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_io_imu.h"

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Component_IMU
  *	\brief Componente para IMU.
  *
  *	Este componente é projetado para implementar funções da IMU da aeronave - leitura
  *	e pré-processamento. A IMU suportada é a baseada nos CIs ITG3205 e ADXL345, mas
  *	outros modelos podem ser incorporados via #define.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef C_IO_IMU_USE_ITG_ADXL_HMC
	#define GYRO_ADDR   0x68 // The address of ITG3205
	#define ACCL_ADDR   0x53 // The address of ADXL345
	#define MAGN_ADDR   0x1E // The address of HMC5883
	#define GYRO_X_ADDR 0x1D // Start address for x-axis
	#define ACCL_X_ADDR 0x32 // Start address for x-axis
	#define MAGN_X_ADDR 0x03 // Start address for x-axis
#else
	#error "Define an IMU type in `c_io_imu.h`! C_IO_USE_ITG_ADXL, or other!"
#endif

#define ROLL        0
#define PITCH       1
#define YAW         2

#define X           0
#define Y           1
#define Z           2

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t imuBuffer[8];
unsigned char ACCL_ID = 0;
unsigned char GYRO_ID = 0;
unsigned char MAGN_ID = 0;

float gyro_rpy[3], acce_rpy[3], filt_rpy[3];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializa a IMU.
 *
 * Seta sensibilidade do acelerômetro e liga o girscópio.
 */
void c_io_imu_init() {

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC // Inicialização para a IMU selecionada
	// Get Accelerometer ID
	c_common_i2c_readBytes(ACCL_ADDR, 0x00, 1, &ACCL_ID);

	// Accelerometer increase G-range (+/- 16G)
	c_common_i2c_writeByte(ACCL_ADDR, 0x31, 0b00001011);

    //  ADXL345 (Accel) POWER_CTL
    c_common_i2c_writeByte(ACCL_ADDR, 0x2D, 0);
    c_common_i2c_writeByte(ACCL_ADDR, 0x2D, 16);
    c_common_i2c_writeByte(ACCL_ADDR, 0x2D, 8);

    // Gyro ID and setup
	c_common_i2c_readBytes(GYRO_ADDR, 0x00, 1, &GYRO_ID);
	c_common_i2c_writeByte(GYRO_ADDR, 22, 24);

    // HMC5883 (Magn) Run in continuous mode
    c_common_i2c_writeByte(MAGN_ADDR, 0x02, 0x00);
#endif

}

/** \brief Obtem as leituras raw do acelerômetro, giro e magnetômetro.
 *
 *
 */
void c_io_imu_getRaw(int * accRaw, int * gyroRaw, int * magRaw) {
    // Read x, y, z acceleration, pack the data.
	c_common_i2c_readBytes(ACCL_ADDR, ACCL_X_ADDR, 6, imuBuffer);
    accRaw[0] = ~(((char)imuBuffer[0] | ((char)imuBuffer[1] << 8))-1);
    accRaw[1] = ~(((char)imuBuffer[2] | ((char)imuBuffer[3] << 8))-1);
    accRaw[2] = ~(((char)imuBuffer[4] | ((char)imuBuffer[5] << 8))-1);

    // Read x, y, z from gyro, pack the data
	c_common_i2c_readBytes(GYRO_ADDR, GYRO_X_ADDR, 6, imuBuffer);
    gyroRaw[0] =  (int)imuBuffer[1] | ((int)imuBuffer[0] << 8);
    gyroRaw[1] = ((int)imuBuffer[3] | ((int)imuBuffer[2] << 8)) * -1;
    gyroRaw[2] = ((int)imuBuffer[5] | ((int)imuBuffer[4] << 8)) * -1;

    // Read x, y, z from magnetometer;
    c_common_i2c_readBytes(MAGN_ADDR, MAGN_X_ADDR, 6, imuBuffer);
    for (unsigned char i =0; i < 3; i++) {
       magRaw[i] = (int)imuBuffer[(i * 2) + 1] | ((int)imuBuffer[i * 2] << 8);
    }
}

/** \brief Retorna os ângulos RPY através de um filtro complementar simples.
 *
 *
 */
void c_io_imu_getComplimentaryRPY(float * rpy) {
	int acce_raw[3], gyro_raw[3], magn_raw[3];

	c_io_imu_getRaw(acce_raw, gyro_raw, magn_raw);

	float32_t px, py, pz;

	//arm_sqrt_f32((float32_t)pow(acce_raw[Z],2), pz);

	acce_rpy[ROLL ] = atan2( acce_raw[X], sqrt(pow(acce_raw[Y],2) + pow(acce_raw[Z],2)));// - mean_acce_rpy[ROLL] ;
	acce_rpy[PITCH] = atan2( acce_raw[Y], sqrt(pow(acce_raw[X],2) + pow(acce_raw[Z],2)));// - mean_acce_rpy[PITCH];

	rpy = acce_rpy;
}

/** \brief Calibra a IMU considerando o veículo em repouso.
 *
 *
 */
void c_io_imu_calibrate() {

}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

