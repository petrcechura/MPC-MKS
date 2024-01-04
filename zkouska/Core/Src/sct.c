/*
 * sct.c
 *
 *  Created on: Oct 12, 2023
 *      Author: xcechu00
 */

#include "main.h"
#include "sct.h"

static const uint32_t reg_values[3][10] = {
		{
				//PCDE--------GFAB @ DIS1
				0b0111000000000111 << 16,
				0b0100000000000001 << 16,
				0b0011000000001011 << 16,
				0b0110000000001011 << 16,
				0b0100000000001101 << 16,
				0b0110000000001110 << 16,
				0b0111000000001110 << 16,
				0b0100000000000011 << 16,
				0b0111000000001111 << 16,
				0b0110000000001111 << 16,
		},

		{
				//----PCDEGFAB---- @ DIS2
				0b0000011101110000 << 0,
				0b0000010000010000 << 0,
				0b0000001110110000 << 0,
				0b0000011010110000 << 0,
				0b0000010011010000 << 0,
				0b0000011011100000 << 0,
				0b0000011111100000 << 0,
				0b0000010000110000 << 0,
				0b0000011111110000 << 0,
				0b0000011011110000 << 0,
		},

		{
				//PCDE--------GFAB @ DIS3
				0b0111000000000111 << 0,
				0b0100000000000001 << 0,
				0b0011000000001011 << 0,
				0b0110000000001011 << 0,
				0b0100000000001101 << 0,
				0b0110000000001110 << 0,
				0b0111000000001110 << 0,
				0b0100000000000011 << 0,
				0b0111000000001111 << 0,
				0b0110000000001111 << 0,
		}
};

/* FINAL EXAM */
static const uint32_t cube_values[3][4] = { // [display][digit]
		{
				0b0000000000001000 << 16,  // 010
				0b0010000000000000 << 16,  // 100
				0b0000000000000010 << 16,  // 001
				0b0010000000000010 << 16   // 101
		},

		{
				0b0000000010000000 << 0,
				0b0000001000000000 << 0,
				0b0000000000100000 << 0,
				0b0000001000100000 << 0
		},

		{
				0b0000000000001000 << 0,
				0b0010000000000000 << 0,
				0b0000000000000010 << 0,
				0b0010000000000010 << 0
		}

};


void sct_init()
{
	sct_led(0);
}


void sct_led(uint32_t value)
{
	for (uint16_t i = 0; i < 32; i++)  {
		HAL_GPIO_WritePin(SCT_SDI_GPIO_Port, SCT_SDI_Pin, (value & 1));
		HAL_GPIO_WritePin(SCT_CLK_GPIO_Port, SCT_CLK_Pin, 1);
		HAL_GPIO_WritePin(SCT_CLK_GPIO_Port, SCT_CLK_Pin, 0);
		value = value >> 1;
	}

	HAL_GPIO_WritePin(SCT_NLA_GPIO_Port, SCT_NLA_Pin, 1);
	HAL_GPIO_WritePin(SCT_NLA_GPIO_Port, SCT_NLA_Pin, 0);

}

// print the actual value (=converts to bits) on the led display
void sct_value(uint16_t value)
{
	uint32_t reg = 0;
	reg |= reg_values[0][value / 100 % 10]; // hundred
	reg |= reg_values[1][value / 10 % 10];  // tens
	reg |= reg_values[2][value / 1 % 10];   // one

	sct_led(reg);
}

void sct_cube(uint8_t value)
{
	uint32_t reg = 0;


	switch(value) {

	case 1:
		reg |= cube_values[1][0];
		break;
	case 2:
		reg |= cube_values[0][1];
		reg |= cube_values[2][2];
		break;
	case 3:
		reg |= cube_values[1][0];
		reg |= cube_values[0][1];
		reg |= cube_values[2][2];
		break;
	case 4:
		reg |= cube_values[0][3];
		reg |= cube_values[2][3];
		break;
	case 5:
		reg |= cube_values[0][3];
		reg |= cube_values[2][3];
		reg |= cube_values[1][0];
		break;
	case 6:
		reg |= cube_values[0][3];
		reg |= cube_values[1][3];
		reg |= cube_values[2][3];
		break;
	default:
		reg |= cube_values[0][0];
		reg |= cube_values[1][3];
		reg |= cube_values[2][0];
		break;

	}

	sct_led(reg);

}
