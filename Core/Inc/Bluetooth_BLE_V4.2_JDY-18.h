/**
 *      Data: 31/10/2022
 *      Universidade
 *      Autores: Pedro Otávio Fonseca Pires e Leandro Guatimosim
 *      Versão: 1.0
 *      Licença: MIT
 *      Nome da API: API para usode um Módulo Bluetooth BLE V4.2 JDY-18
 *      Arquivos relacionados: Bluetooth_BLE_V4.2_JDY-18.h
 *      					   Bluetooth_BLE_V4.2_JDY-18.c
 *      Resumo: Esta API foi desenvolvida como trabalho da disciplina de
 *Programação de Sistemas Embarcados da UFMG – Prof. Ricardo de Oliveira Duarte
 *– Departamento de Engenharia Eletrônica
 *------------------------------------------------------------------------------------------------------------------------------------------------------
 *      @file Bluetooth_BLE_V4.2_JDY-18.h
 *      @author Pedro Otávio Fonseca Pires and Leandro Guatimosim Gripp
 *      @brief This is the header file for the API of the
 *Bluetooth_BLE_V4.2_JDY-18 module. Datasheet:
 *https://manuals.plus/bluetooth-module/bluetooth-jdy-18-4-2-ble-module-usage-manual#axzz7jKHPWZQl
 */

#ifndef INC_BLUETOOTH_BLE_V4_2_JDY_18_H_
#pragma once

#include "stm32f1xx_hal.h"

#define INC_BLUETOOTH_BLE_V4_2_JDY_18_H_

#define MAX_INSTRUCTION_SIZE 50

typedef enum {
	SET_NAME,
	SET_ROLE,
	SET_BAUD,
	SET_BEACON_UUID,
	SET_TRANSMITTING_POWER,
	MASTER_SCAN_FOR_SLAVES,
	MASTER_CONNECT_SLAVE,
	SET_PERMISSIONS
} AtInstruction_t;

typedef enum {
	BAUD_1200 = 1,
	BAUD_2400 = 2,
	BAUD_4800 = 3,
	BAUD_9600 = 4,
	BAUD_19200 = 5,
	BAUD_38400 = 6,
	BAUD_57600 = 7,
	BAUD_115200 = 8,
	BAUD_230400 = 9
} BaudRate_t;

typedef enum {
	SLAVE = 0,
	MASTER = 1,
	IBEACON = 3
} Role_t;

typedef struct {
	int index;
	char *mac;
	int signalStrength;
	char *name;
} Device_t;

typedef struct {
	int nbOfDevices;
	Device_t *devices;
} ListDevices_t;

void setupBLE(UART_HandleTypeDef *huartInterface, UART_HandleTypeDef *loggingInterface);
//void setupBLE(UART_HandleTypeDef *huartInterface);
void setName (char *name);
void setRole (Role_t role);
void setBaud (BaudRate_t baud);
void setBeaconUuid (char *uuidInHex);
void setBeaconTramsmittingPower (int powerPercentage);
ListDevices_t masterScanForSlaves ();
void connectMasterToSlave (int index);
void connectMasterToSlaveFromMACAddress (char *mac);

#endif /* INC_BLUETOOTH_BLE_V4_2_JDY_18_H_ */
