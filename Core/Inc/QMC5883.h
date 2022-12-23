/*
 * QMC5883.h
 *
 *  Created on: 11 May 2021
 *      Author: Serdar
 */

#ifndef QMC5883_H_
#define QMC5883_H_
//#########################################################################################################
#ifdef __cplusplus
 extern "C" {
#endif
//#########################################################################################################
#include "main.h"
#include"stm32f1xx.h"
 //#########################################################################################################
#define Standby 0
#define Continuous 1
#define QMC_OK 0
#define QMC_FALSE 1
#define ADDR_REG_A 0x00
#define ADDR_REG_B 0x01
#define ADDR_REG_MODE 0x02
#define ADDR_REG_STATUS 0x09
//#########################################################################################################

typedef struct QMC
{
	I2C_HandleTypeDef   *i2c;
	uint8_t				ADDR_Control_RegisterA;
	uint8_t				ADDR_Control_RegisterB;
	uint8_t				ADDR_Mode_Register;
	uint8_t				ADDR_Status_Register;
	uint8_t				Control_RegisterA;
	uint8_t				Control_RegisterB;
	uint8_t				Mode_Register;
	uint8_t				Status_Register;
	uint8_t             datas[6];
	int16_t             Xaxis;
	int16_t             Yaxis;
	int16_t             Zaxis;
	float			    heading;
	float               compas;
}QMC_t;

typedef enum{
	Normal,
	Positive,
	Negative
} Measurement_Mode;

typedef enum{
	one,
	two,
	four,
	eight
} Samples;

typedef enum{
	_0_88,
	_1_3,
	_1_9,
	_2_5,
	_4_0,
	_4_7,
	_5_6,
	_8_1
} Gain;

typedef enum{
	Continuous_meas,
	Single_meas,
	Idle,
} Op_mode;

typedef struct{
	Measurement_Mode meas_mode;
	Samples samples_num;
	uint8_t output_rate;
	Gain	gain;
	Op_mode op_mode;
} Compass_config;

//#########################################################################################################
uint8_t QMC_init(QMC_t *qmc,I2C_HandleTypeDef *i2c, Compass_config *config);
uint8_t QMC_read(QMC_t *qmc);
float   QMC_readHeading(QMC_t *qmc);
uint8_t QMC_Standby(QMC_t *qmc);
uint8_t QMC_Reset(QMC_t *qmc);





//#########################################################################################################
#ifdef __cplusplus
}
#endif
#endif /* QMC5883_H_ */
