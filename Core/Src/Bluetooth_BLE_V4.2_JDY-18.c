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
 *      @file Bluetooth_BLE_V4.2_JDY-18.c
 *      @author Pedro Otávio Fonseca Pires and Leandro Guatimosim Gripp
 *      @brief This is the source file for the API of the
 *Bluetooth_BLE_V4.2_JDY-18 module. Datasheet:
 *https://manuals.plus/bluetooth-module/bluetooth-jdy-18-4-2-ble-module-usage-manual#axzz7jKHPWZQl
 */

#include "Bluetooth_BLE_V4.2_JDY-18.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

char *atInstructions[] = {
	"AT+NAME",
	"AT+ROLE",
	"AT+BAUD",
	"AT+IBUUID",
	"AT+POWR",
	"AT+INQ",
	"AT+CONN",
	"AT+PERM"
};

UART_HandleTypeDef *huart;
UART_HandleTypeDef *loggingHuart;

void sendInstruction (AtInstruction_t instruction, char* parameter);
char** str_split (char* a_str, const char a_delim);
int getSubstring (char *source, char *target,int from, int to);
void sendToLogger (char *msg);

void setupBLE(UART_HandleTypeDef *huartInterface, UART_HandleTypeDef *loggingInterface) {
	huart = huartInterface;
	loggingHuart = loggingInterface;
	sendInstruction(SET_PERMISSIONS, "11111");
}

void sendInstruction (AtInstruction_t instruction, char* parameter) {
	char *instructionPrefix = atInstructions[instruction];
	char *completeInstruction = (char*) malloc(30 * sizeof(char));
	sprintf(completeInstruction, "%s%s\r\n", instructionPrefix, parameter);
	sendToLogger(completeInstruction);
	HAL_UART_Transmit(huart, (uint8_t *) completeInstruction, strlen(completeInstruction), HAL_MAX_DELAY);
	free(completeInstruction);
}

void setName (char *name) {
	sendInstruction(SET_NAME, name);
}

void setRole (Role_t role) {
	char roleString[2];
	itoa(role, roleString, 10);
	sendInstruction(SET_ROLE, roleString);
}

void setBaud (BaudRate_t baud) {
	char baudString[2];
	itoa(baud, baudString, 10);
	sendInstruction(SET_BAUD, baudString);
}

void setBeaconUuid (char *uuidInHex) {
	sendInstruction(SET_BEACON_UUID, uuidInHex);
}

void setBeaconTramsmittingPower (int powerPercentage) {
	if(powerPercentage > 100) powerPercentage = 100;
	if(powerPercentage < 0) powerPercentage = 0;
	float power = powerPercentage/100.0;
	char powerString[5];
	gcvt(power, 2, powerString);
	sendInstruction(SET_TRANSMITTING_POWER, powerString);
}

ListDevices_t masterScanForSlaves () {

	char inputBuffer[1];
	char *allText = (char *) malloc(1000 * sizeof(char));
	// Send the scan request
	sendToLogger("About to inquire \r\n");
	sendInstruction(MASTER_SCAN_FOR_SLAVES, "");
	sendToLogger("Inquired \r\n");
	// Receive the scan answer
	while(!((strstr(allText, "STOP:SCAN") != NULL))) {
		HAL_UART_Receive(huart, (uint8_t *) inputBuffer, 1, HAL_MAX_DELAY);
		strcat(allText, inputBuffer);
	}
	sendToLogger("Received \r\n");
	sendToLogger(allText);
	sendToLogger("\r\nRetransmitted \r\n");
	// Break the scan into lines:
	char** lines = str_split(allText, '\n');

	// Breaking the lines into tokens and creating the struct
	size_t nbOfEntries = 0;
	while (*(lines + nbOfEntries) != 0) nbOfEntries++;
	nbOfEntries--;
	Device_t *entries = (Device_t *) malloc(nbOfEntries * sizeof(Device_t));
	for (int i = 0; i < nbOfEntries; i++) {
		// Removing leading trash characters
		char *line = lines[i];
		char *e = strrchr(line, '+');
		int lastPlusIndex =  (int)(e - line);
		getSubstring(line, line, lastPlusIndex, strlen(line));
		char **tokens = str_split(line, ',');
		// Checking if all the needed info is available.
		if (*(tokens + 2) != 0) {
			getSubstring(tokens[0], tokens[0], 7, strlen(tokens[0]));
			getSubstring(tokens[2], tokens[2], 0, strlen(tokens[2])-2);

			entries[i].index = i+1;
			entries[i].mac = tokens[0];
			entries[i].signalStrength = atoi(tokens[1]);
			entries[i].name = tokens[2];
		} else {
			entries[i].index = -1;
			entries[i].mac = "INVALID";
			entries[i].signalStrength = 1;
			entries[i].name = "INVALID";
		}
	}
	free(allText);
//	return entries;
	ListDevices_t answer = {nbOfEntries, entries};
	return answer;
}

void connectMasterToSlave (int index) {
	char indexString[2];
	itoa(index, indexString, 10);
	sendInstruction(MASTER_CONNECT_SLAVE, indexString);
}

void connectMasterToSlaveFromMACAddress (char *mac) {
	sendInstruction(MASTER_CONNECT_SLAVE, mac);
}

char** str_split(char* a_str, const char a_delim)
{
    char** result    = 0;
    size_t count     = 0;
    char* tmp        = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;

    /* Count how many elements will be extracted. */
    while (*tmp)
    {
        if (a_delim == *tmp)
        {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }

    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);

    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
    count++;

    result = malloc(sizeof(char*) * count);

    if (result)
    {
        size_t idx  = 0;
        char* token = strtok(a_str, delim);

        while (token)
        {
            assert(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
        }
        assert(idx == count - 1);
        *(result + idx) = 0;
    }

    return result;
}

int  getSubstring(char *source, char *target,int from, int to)
{
	int length=0;
	int i=0,j=0;

	//get length
	while(source[i++]!='\0')
		length++;

	if(from<0 || from>length){
		printf("Invalid \'from\' index\n");
		return 1;
	}
	if(to>length){
		printf("Invalid \'to\' index\n");
		return 1;
	}

	for(i=from,j=0;i<=to;i++,j++){
		target[j]=source[i];
	}

	//assign NULL at the end of string
	target[j]='\0';

	return 0;
}

void sendToLogger (char *msg) {
	HAL_UART_Transmit(loggingHuart, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);
}
