#include "mt6835.h"

uint32_t mt6835_get_position(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin) {
	uint32_t angulo = 0;
	uint8_t tx_data[6] = {0};
	uint8_t rx_data[6] = {0};

	tx_data[0] = MT6835_BURST_READ << 4;
	tx_data[1] = MT6835_ANGLE_HIGH;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 6, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	angulo = ((uint32_t)rx_data[2] << 16) |
			 ((uint32_t)rx_data[3] << 8)  |
			 ((uint32_t)rx_data[4]);

	return angulo;
}

uint16_t mt6835_get_abz_res(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin) {
	uint16_t abz_res = 0;
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ABZ_RES_HIGH;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	abz_res = (uint16_t)rx_data[2] << 8;

	// Avanzo al registro LOW
	tx_data[1] = MT6835_ABZ_RES_LOW;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	abz_res |= rx_data[2];

	abz_res >>= 2;

	return abz_res + 1;
}

MT6835_RETURNS_t mt6835_set_abz_res(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin, uint16_t abz_res) {
	if (abz_res > 16384 || abz_res == 0) {
		return MT6835_INV_RES;
	}

	uint8_t tx_data[3] = {0};
	// Leo abz_off y swap para no pisarlos
	uint8_t abz_off = mt6835_get_abz_off(mt6835, CS_port, CS_pin);
	uint8_t abz_swap = mt6835_get_abz_swap(mt6835, CS_port, CS_pin);

	tx_data[0] = MT6835_WRITE << 4;
	tx_data[1] = MT6835_ABZ_RES_HIGH;
	tx_data[2] = ((abz_res - 1) >> 6) & 0x00FF;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(mt6835, tx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	// Ahora escribo byte low
	tx_data[1] = MT6835_ABZ_RES_LOW;

	uint8_t temp = ((abz_res - 1) << 2) & 0x00FF;
	temp = (abz_off == 1) ? (temp | 0x02) : (temp & 0xFD);
	temp = (abz_swap == 1) ? (temp | 0x01) : (temp & 0xFE );

	tx_data[2] = temp;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(mt6835, tx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	return MT6835_RES_SET;
}

uint8_t mt6835_get_abz_off(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ABZ_RES_LOW;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	return (rx_data[2] >> 1) & 0x01;
}

void mt6835_set_abz_off(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin, MT6835_ABZ_OFF_t abz_off) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	// Primero leo para no pisar bits de abz_res y abz_swap
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ABZ_RES_LOW;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	uint8_t temp = rx_data[2];

	// Si abz_off > 0 pongo el bit en 1
	temp = (abz_off > 0) ? (temp | 0x02) : (temp & 0xFD);

	// Ahora escribo registro completo con nuevo valor de abz_off
	tx_data[0] = MT6835_WRITE << 4;
	tx_data[2] = temp;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(mt6835, tx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);
}

uint8_t mt6835_get_abz_swap(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ABZ_RES_LOW;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	return rx_data[2] & 0x01;
}

void mt6835_set_abz_swap(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin, MT6835_ABZ_SWAP_t abz_swap) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	// Primero leo para no pisar bits de abz_res y abz_off
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ABZ_RES_LOW;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	uint8_t temp = rx_data[2];

	// Si abz_swap > 0 pongo el bit en 1
	temp = (abz_swap > 0) ? (temp | 0x01) : (temp & 0xFE);

	// Ahora escribo registro completo con nuevo valor de abz_off
	tx_data[0] = MT6835_WRITE << 4;
	tx_data[2] = temp;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(mt6835, tx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);
}

MT6835_RETURNS_t mt6835_set_zero(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = (MT6835_SET_ZERO << 4);

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	if (rx_data[2] != 0x55) {
		return MT6835_ZERO_FAIL;
	}

	return MT6835_ZERO_OK;
}

uint8_t mt6835_get_z_edge(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ZERO_LOW;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	return (rx_data[2] >> 3) & 0x01;
}

void mt6835_set_z_edge(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin, MT6835_Z_EDGE_t z_edge) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	// Primero leo para no pisar zero_low y z_width
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ZERO_LOW;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	uint8_t temp = rx_data[2];

	// Pongo el bit en 1 cuando z_edge > 0
	temp = (z_edge > 0) ? (temp | 0x08) : (temp & 0xF7);

	tx_data[0] = MT6835_WRITE << 4;
	tx_data[2] = temp;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(mt6835, tx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);
}

uint8_t mt6835_get_z_width(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ZERO_LOW;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	return rx_data[2] & 0x07;
}

MT6835_RETURNS_t mt6835_set_z_width(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin, MT6835_Z_WIDTH_t z_width) {
	if (z_width > 7) {
		return MT6835_INV_WIDTH;
	}

	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	// Primero leo para no pisar zero_low y z_edge
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ZERO_LOW;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	uint8_t temp = rx_data[2] & 0xF8;

	temp |= z_width;

	tx_data[0] = MT6835_WRITE << 4;
	tx_data[2] = temp;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(mt6835, tx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	return MT6835_WIDTH_OK;
}

uint8_t mt6835_get_z_phase(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin) {
	uint8_t tx_data[3];
	uint8_t rx_data[3];

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_Z_PHASE;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	return (rx_data[2] >> 6) & 0x03;
}

MT6835_RETURNS_t mt6835_set_z_phase(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin, MT6835_Z_PHASE_t z_phase) {
	if (z_phase > 3) {
		return MT6835_INV_PHASE;
	}

	uint8_t tx_data[3];
	uint8_t rx_data[3];

	// Primero leo para no pisar UVW config
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_Z_PHASE;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	uint8_t temp = rx_data[2] & 0x3F;

	temp |= (z_phase << 6);

	tx_data[0] = MT6835_WRITE << 4;
	tx_data[2] = temp;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(mt6835, tx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	return MT6835_PHASE_OK;
}

uint8_t mt6835_get_rot_dir(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ROT_DIR;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	return (rx_data[2] >> 3) & 0x01;
}

void mt6835_set_rot_dir(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin, MT6835_ROT_DIR_t rot_dir) {
	uint8_t tx_data[3] = {0};
	uint8_t rx_data[3] = {0};

	// Primero leo para no pisar bits del registro
	tx_data[0] = MT6835_READ << 4;
	tx_data[1] = MT6835_ROT_DIR;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mt6835, tx_data, rx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);

	uint8_t temp = rx_data[2] & 0xF7;

	// Si rot_dir > 0 pongo registro en 1: A lead B en CCW
	temp = (rot_dir > 0) ? (temp | 0x08) : (temp & 0xF7);

	tx_data[0] = MT6835_WRITE << 4;
	tx_data[2] = temp;

	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(mt6835, tx_data, 3, 1000);
	HAL_GPIO_WritePin(CS_port, CS_pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef init_encoder(SPI_HandleTypeDef *mt6835, GPIO_TypeDef *CS_port, uint16_t CS_pin, encoder_t *encoder,
	    		  	  	  	   uint16_t abz_res, MT6835_ABZ_SWAP_t abz_swap, MT6835_Z_EDGE_t z_edge,
							   MT6835_Z_WIDTH_t z_width, MT6835_Z_PHASE_t z_phase, MT6835_ROT_DIR_t rot_dir) {
	MT6835_RETURNS_t err;

	// Configuro resolucion modo ABZ
	err = mt6835_set_abz_res(mt6835, CS_port, CS_pin, abz_res);

	if (err != MT6835_RES_SET || mt6835_get_abz_res(mt6835, CS_port, CS_pin) != abz_res) {
		encoder->abz_res = UINT16_MAX;
		return HAL_ERROR;
	}

	encoder->abz_res = abz_res;

	// Enciendo modo ABZ
	mt6835_set_abz_off(mt6835, CS_port, CS_pin, 0);

	if (mt6835_get_abz_off(mt6835, CS_port, CS_pin) != 0) {
		encoder->abz_off = UINT8_MAX;
		return HAL_ERROR;
	}

	encoder->abz_off = 0;

	// Configuro intercambio de canales AB
	mt6835_set_abz_swap(mt6835, CS_port, CS_pin, abz_swap);

	if (mt6835_get_abz_swap(mt6835, CS_port, CS_pin) != abz_swap) {
		encoder->abz_swap = UINT8_MAX;
		return HAL_ERROR;
	}

	encoder->abz_swap = abz_swap;

	// Configuro alineacion de los flanco de Z con 0
	mt6835_set_z_edge(mt6835, CS_port, CS_pin, z_edge);

	if (mt6835_get_z_edge(mt6835, CS_port, CS_pin) != z_edge) {
		encoder->z_edge = UINT8_MAX;
		return HAL_ERROR;
	}

	encoder->z_edge = z_edge;

	// Configuro ancho de pulso de Z
	err = mt6835_set_z_width(mt6835, CS_port, CS_pin, z_width);

	if (err != MT6835_WIDTH_OK || mt6835_get_z_width(mt6835, CS_port, CS_pin) != z_width) {
		encoder->z_width = UINT8_MAX;
		return HAL_ERROR;
	}

	encoder->z_width = z_width;

	// Configuro fase de Z
	err = mt6835_set_z_phase(mt6835, CS_port, CS_pin, z_phase);

	if (err != MT6835_PHASE_OK || mt6835_get_z_phase(mt6835, CS_port, CS_pin) != z_phase) {
		encoder->z_phase = UINT8_MAX;
		return HAL_ERROR;
	}

	encoder->z_phase = z_phase;

	// Configuro orden de canales AB al girar
	mt6835_set_rot_dir(mt6835, CS_port, CS_pin, rot_dir);

	if (mt6835_get_rot_dir(mt6835, CS_port, CS_pin) != rot_dir) {
		encoder->rot_dir = UINT8_MAX;
		return HAL_ERROR;
	}

	encoder->rot_dir = rot_dir;

	encoder->init = 1;

	return HAL_OK;
}













