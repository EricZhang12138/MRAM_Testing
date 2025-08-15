/*
 * March_C_read_write.c
 *
 *  Created on: Aug 15, 2025
 *      Author: YZhang
 */
#include "main.h"

void WriteMem(uint8_t chip, uint32_t addr, uint16_t length, uint8_t* data){
	length += 4; //Account for the header
	uint16_t selectedPin = 0;
	switch(chip){
	case 0:
		selectedPin = 1024;
		break;
	case 1:
		selectedPin = 4096;
		break;
	case 2:
		selectedPin = 16384;
		break;
	case 3:
		selectedPin = 32768;
		break;
	case 4:
		selectedPin = 512;
		break;
	case 5:
		selectedPin = 2048;
		break;
	}

	/*if(chip>1){ //If it's FRAM we need to re-enable writes after every write
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET); //Set WRite ENable
		  HAL_SPI_Transmit(&hspi1, &wren, 1, 10);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
	}*/

	uint8_t TxBuff[length];
	TxBuff[0] = 0x02;
	TxBuff[1] = (addr>>16)&0x0000FF;
	TxBuff[2] = (addr>>8)&0x0000FF;
	TxBuff[3] = (addr)&0x0000FF;                                                                   //we only need the first 24 bits (16Mb)

	for(int i=4; i<length; i++){
		TxBuff[i] = *(data+(i-4)); //Loop through all the data and put it into the Tx buffer.
	}

	HAL_GPIO_WritePin(GPIOE, selectedPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)TxBuff, length, 100);
	HAL_GPIO_WritePin(GPIOE, selectedPin, GPIO_PIN_SET);
}

void ReadMem(uint8_t chip, uint32_t addr, uint16_t length, uint8_t* buffer){
	length += 4; //Account for the header

	uint16_t selectedPin = 0;
	switch(chip){
	case 0:
		selectedPin = 1024;
		break;
	case 1:
		selectedPin = 4096;
		break;
	case 2:
		selectedPin = 16384;
		break;
	case 3:
		selectedPin = 32768;
		break;
	case 4:
		selectedPin = 512;
		break;
	case 5:
		selectedPin = 2048;
		break;
	}

	uint8_t TxBuff[length];
	TxBuff[0] = 0x03;
	TxBuff[1] = (addr>>16)&0x0000FF;
	TxBuff[2] = (addr>>8)&0x0000FF;
	TxBuff[3] = (addr)&0x0000FF;
	for(int j = 4; j< length; j++){
		TxBuff[j] = '\0';
	}
	uint8_t RxBuff[length];

	HAL_GPIO_WritePin(GPIOE, selectedPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)TxBuff, (uint8_t *)RxBuff, length, length);
	HAL_GPIO_WritePin(GPIOE, selectedPin, GPIO_PIN_SET);

	for(int i=4; i<length; i++){
		*(buffer+(i-4)) = RxBuff[i]; //Load all the received data into the proper location
	}

}
