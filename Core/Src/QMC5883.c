/*
 * QMC5883.c
 *
 *  Created on: 11 May 2021
 *      Author: Serdar
 */
#include"QMC5883.h"
#include "math.h"

//###############################################################################################################
uint8_t QMC_init(QMC_t *qmc,I2C_HandleTypeDef *i2c, Compass_config *config)
{
	uint8_t array[3];
	qmc->i2c=i2c;
	qmc->Control_RegisterA=0x00;
	qmc->Control_RegisterB=0x00;
	qmc->Mode_Register=0x00;
	array[0]=qmc->Control_RegisterA;
	array[1]=qmc->Control_RegisterB;
	array[2]=qmc->Mode_Register;

	if(config->output_rate ==0.75);
	else if(config->output_rate==1.5)qmc->Control_RegisterA 		|= 0b00000100;
	else if(config->output_rate==3)qmc->Control_RegisterA 			|= 0b00001000;
	else if(config->output_rate==7.5)qmc->Control_RegisterA 		|= 0b00001100;
	else if(config->output_rate==15)qmc->Control_RegisterA 			|= 0b00010000;
	else if(config->output_rate==30)qmc->Control_RegisterA 			|= 0b00010100;
	else if(config->output_rate==75)qmc->Control_RegisterA 			|= 0b00011000;
	else qmc->Control_RegisterA |= 0b00010000;

	if(config->meas_mode == Normal);
	else if (config->meas_mode == Positive) qmc->Control_RegisterA 	|= 0b00000001;
	else if (config->meas_mode == Negative) qmc->Control_RegisterA 	|= 0b00000010;

	if(config->samples_num == one);
	else if(config->samples_num == two) qmc->Control_RegisterA 		|= 0b00100000;
	else if(config->samples_num == four) qmc->Control_RegisterA 	|= 0b01000000;
	else if(config->samples_num == eight) qmc->Control_RegisterA 	|= 0b01100000;

	if(config->gain == _0_88);
	else if (config->gain == _1_3) 	qmc->Control_RegisterB 			|= 0b00100000;
	else if (config->gain == _1_9) 	qmc->Control_RegisterB 			|= 0b01000000;
	else if (config->gain == _2_5) 	qmc->Control_RegisterB 			|= 0b01100000;
	else if (config->gain == _4_0) 	qmc->Control_RegisterB 			|= 0b10000000;
	else if (config->gain == _4_7) 	qmc->Control_RegisterB 			|= 0b10100000;
	else if (config->gain == _5_6) 	qmc->Control_RegisterB 			|= 0b11000000;
	else if (config->gain == _8_1) 	qmc->Control_RegisterB 			|= 0b11100000;
	else qmc->Control_RegisterB 									|= 0b00100000;

	if(config->op_mode == Continuous_meas);
	else if(config->op_mode == Single_meas) qmc->Mode_Register 		|= 0b00000001;
	else if(config->op_mode == Idle) qmc->Mode_Register 			|= 0b00000010;
	else qmc->Mode_Register 										|= 0b00000001;

	if(HAL_I2C_Mem_Write(qmc->i2c, 0x3D, qmc->ADDR_Control_RegisterA, 1, &array[0], 1, 100)!=HAL_OK) return 1;
	if(HAL_I2C_Mem_Write(qmc->i2c, 0x3D, qmc->ADDR_Control_RegisterB, 1, &array[1], 1, 100)!=HAL_OK) return 1;
	if(HAL_I2C_Mem_Write(qmc->i2c, 0x3D, qmc->ADDR_Mode_Register, 1, &array[2], 1, 100)!=HAL_OK) return 1;

	return 0;
}

uint8_t QMC_read(QMC_t *qmc)
{
	  qmc->datas[0]=0;
	  HAL_I2C_Mem_Read(qmc->i2c, 0x3D, qmc->ADDR_Status_Register, 1, qmc->datas, 1, 100);

	  if((qmc->datas[0]&0x01)==1)
	  {
		  HAL_I2C_Mem_Read(qmc->i2c, 0x3D, 0x03, 1, qmc->datas, 6, 100);
		  qmc->Xaxis= (qmc->datas[0]<<8) | qmc->datas[1];
		  qmc->Yaxis= (qmc->datas[2]<<8) | qmc->datas[3];
		  qmc->Zaxis= (qmc->datas[4]<<8) | qmc->datas[5];

		  qmc->compas=atan2f(qmc->Yaxis,qmc->Xaxis)*180.00/M_PI;

		  if(qmc->compas>0)
		  {
			  qmc->heading= qmc->compas;
		  }
		  else
		  {
			  qmc->heading=360+qmc->compas;
		  }
	  }
	  else
	  {
		  return 1;
	  }
return 0;
}

float QMC_readHeading(QMC_t *qmc)
{
	QMC_read(qmc);
	return qmc->heading;
}

uint8_t QMC_Standby(QMC_t *qmc)
{
	uint8_t array[1]={0};
	if(HAL_I2C_Mem_Write(qmc->i2c, 0x1A, 0x09, 1, &array[0], 1, 100)!=HAL_OK)return 1;
	return 0;
}
uint8_t QMC_Reset(QMC_t *qmc)
{
	uint8_t array[1]={0x80};
	if(HAL_I2C_Mem_Write(qmc->i2c, 0x1A, 0x0A, 1, &array[0], 1, 100)!=HAL_OK)return 1;
	return 0;
}
