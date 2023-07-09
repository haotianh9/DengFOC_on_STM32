/*
 * as5048a.c
 *
 *  Created on: Jun 8, 2023
 *      Author: hht
 */

#include "as5048a.h"
uint8_t spiCalcEvenParity(uint16_t value){
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

uint16_t read(SPI_HandleTypeDef* _spi, GPIO_TypeDef* _ps, uint16_t _cs,uint16_t registerAddress){

	uint8_t send_data[2];
	uint8_t recv_data[2];
//	uint16_t data2;
	uint16_t command = 0b0100000000000000; // PAR=0 R/W=R
	command = command | registerAddress;

	//Add a parity bit on the the MSB
	command |= ((uint16_t)spiCalcEvenParity(command)<<15);

	//Split the command into two bytes
	send_data[1] = command & 0xFF;
	send_data[0] = ( command >> 8 ) & 0xFF;

	EN_SPI;
	HAL_SPI_Transmit(_spi, (uint8_t *)&send_data, 2, 0xFFFF);
//	HAL_SPI_Transmit(_spi, (uint8_t *)&command, 1, 0xFFFF);
	while (HAL_SPI_GetState(_spi) != HAL_SPI_STATE_READY) {}
	DIS_SPI;
	send_data[0]=0x00;
	send_data[1]=0x00;
	EN_SPI;
	 HAL_SPI_TransmitReceive(_spi,(uint8_t*)&send_data,(uint8_t*)&recv_data,2, 0xFFFF);
//	HAL_SPI_Receive(_spi, (uint8_t *)&recv_data, 2, 0xFFFF);
//	HAL_SPI_Receive(_spi, (uint8_t *)&data2, 1, 0xFFFF);
	while (HAL_SPI_GetState(_spi) != HAL_SPI_STATE_READY) {}
	DIS_SPI;

//	if (recv_data[1] & 0x40) {
//		errorFlag = 1;
//	} else {
//		errorFlag = 0;
//	}

	//Return the data, stripping the parity and error bits
	return (( ( recv_data[1] & 0xFF ) << 8 ) | ( recv_data[0] & 0xFF )) & ~0xC000;
//	return data2 & ~0xC000;
}

