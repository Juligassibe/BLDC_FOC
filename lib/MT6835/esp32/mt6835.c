#include "mt6835.h"

static const char *tag = "MT6835";

esp_err_t mt6835_get_user_id(spi_device_handle_t *mt6835Handle, uint8_t *userID) {
    esp_err_t error;

    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = USER_ID,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);
    
    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al realizar transacción: %s", esp_err_to_name(error));
        return error;
    }

    *userID = operacion.rx_data[0];

    printf("ID leido: %d\n", *userID);

    return ESP_OK;  
}

esp_err_t mt6835_set_user_id(spi_device_handle_t *mt6835Handle, uint8_t userID) {
    esp_err_t error;

    spi_transaction_t operacion = {
        .cmd = WRITE,
        .addr = USER_ID,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    operacion.tx_data[0] = userID;

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al transmitir ID nuevo: %s", esp_err_to_name(error));
        return error;
    }

    printf("ID registrado: %d\n", userID);

    return ESP_OK;
}

esp_err_t mt6835_get_angle(spi_device_handle_t *mt6835Handle, uint32_t *angle) {
    esp_err_t error;
    uint32_t regRx = 0, temp = 0;

    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = ANGLE_HIGH,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    for (int i = 0; i < 4; i++) {
        // Primeras 3 iteraciones leo angulo y en 4ta leo CRC
        error = spi_device_transmit(*mt6835Handle, &operacion);

        if (error != ESP_OK) {
            ESP_LOGE(tag, "Error al leer registro 0x%03X: %s", operacion.addr+i, esp_err_to_name(error));
            return error;
        }

        regRx = operacion.rx_data[0];

        if (i < 3) {
            temp |= (regRx << (16 - 8*i));
        }

        operacion.addr++;
    }

    // Retorno 21 bits de angulo + 3 bits de estado, tener en cuenta al usar
    *angle = temp;

    return ESP_OK;
}

uint8_t calculate_crc(uint32_t angle) {
    // Obtenida de https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/mt6835/MT6835.cpp
    // El CRC usado es distinto de los CRC8 estandar
    uint8_t crc = 0x00;

    uint8_t input = angle>>13;
    crc ^= input;
    for (int k = 8; k > 0; k--)
        crc = (crc & (0x01<<7))?(crc<<1)^0x07:crc<<1;

    input = (angle>>5) & 0xFF;
    crc ^= input;
    for (int k = 8; k > 0; k--)
        crc = (crc & (0x01<<7))?(crc<<1)^0x07:crc<<1;

    input = ((angle<<3) & 0xFF);
    crc ^= input;
    for (int k = 8; k > 0; k--)
        crc = (crc & (0x01<<7))?(crc<<1)^0x07:crc<<1;

    return crc;
}

// NO FUNCA TODAVIA
esp_err_t mt6835_program_eeprom(spi_device_handle_t *mt6835Handle) {
    esp_err_t error;

    spi_transaction_t operacion = {
        .cmd = PROG_EEPROM,
        .addr = 0x000,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);
    
    if (error != ESP_OK || operacion.rx_data[0] != 0x55) {
        ESP_LOGE(tag, "Error al grabar EEPROM: %s", esp_err_to_name(error));
        ESP_LOGE(tag, "ACK: 0x%02X != 0x55", operacion.rx_data[0]);
        return error;
    }

    ESP_LOGI(tag, "EEPROM grabada correctamente, esperar 6s");

    vTaskDelay(pdMS_TO_TICKS(6000));

    return ESP_OK;  
}

esp_err_t mt6835_get_abz_res(spi_device_handle_t *mt6835Handle, uint16_t *abzRes) {
    esp_err_t error;
    uint16_t temp = 0;

    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = ABZ_RES_HIGH,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer resolucion (0x%03X) ABZ: %s", operacion.addr, esp_err_to_name(error));
        return error;
    }

    temp = operacion.rx_data[0];
    temp <<= 8;

    // Primero leo BYTE HIGH (0x007) y luego leo BYTE LOW (0x008)
    operacion.addr = ABZ_RES_LOW;

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer resolucion (0x%03X) ABZ: %s", operacion.addr, esp_err_to_name(error));
        return error;
    }

    // Borro bits del LOW BYTE y luego guardo
    temp &= 0xFF00;
    temp |= operacion.rx_data[0];

    // Elimino bits ABZ_OFF y ABZ_SWAP y +1 debido a que ppr = registro + 1
    *abzRes = ((temp >> 2) + 1) & 0x3FFF;

    printf("Leido ABZ_RES: %d\n", *abzRes);

    return ESP_OK;
}

esp_err_t mt6835_set_abz_res(spi_device_handle_t *mt6835Handle, uint16_t abzRes) {
    if (abzRes == 0) {
        ESP_LOGW(tag, "Resolucion minima: 1 ppr");
        return ESP_FAIL;
    }

    esp_err_t error;

    // Para almacenar ABZ_OFF y ABZ_SWAP
    uint8_t abzOff = 0, abzSwap = 0, temp = 0;

    // Primero leo ABZ_OFF y ABZ_SWAP para no pisar sus valores
    if (mt6835_get_abz_off(mt6835Handle, &abzOff) != ESP_OK) {
        return ESP_FAIL;
    }

    if (mt6835_get_abz_swap(mt6835Handle, &abzSwap) != ESP_OK) {
        return ESP_FAIL;
    }

    spi_transaction_t operacion = {
        .cmd = WRITE,
        .addr = ABZ_RES_HIGH,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    // High byte del registro
    operacion.tx_data[0] = ((abzRes-1) >> 6) & 0x00FF;

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al escribir resolucion (0x%03X) ABZ: %s", operacion.addr, esp_err_to_name(error));
        return error;
    }

    operacion.addr = ABZ_RES_LOW;

    // Debo restar 1 por que los ppr = valor en registro + 1, por ejemplo, si el registro esta todo en 0, es 1 ppr
    temp = ((abzRes-1) << 2) & 0x00FF;
    // Agrego abzOff a ABZ_RES_LOW
    temp = (abzOff == 1) ? (temp | 0x02) : (temp & 0xFD);
    // Agrego abzSwap a ABZ_RES_LOW
    temp = (abzSwap == 1) ? (temp | 0x01) : (temp & 0xFE);
    
    operacion.tx_data[0] = temp;

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al escribir resolucion (0x%03X) ABZ: %s", operacion.addr, esp_err_to_name(error));
        return error;
    }

    printf("Escrito ABZ_RES: %d\n", abzRes);

    return ESP_OK;
}

esp_err_t mt6835_get_abz_off(spi_device_handle_t *mt6835Handle, uint8_t *abzOff) {
    esp_err_t error;

    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = ABZ_RES_LOW,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer ABZ_OFF: %s", esp_err_to_name(error));
        return error;
    }

    *abzOff = (operacion.rx_data[0] & 0x02) >> 1;

    printf("Leido ABZ_OFF: %d\n", *abzOff);

    return ESP_OK;
}

esp_err_t mt6835_set_abz_off(spi_device_handle_t *mt6835Handle, uint8_t abzOff) {
    esp_err_t error;
    uint8_t abzResLow = 0;

    // Primero debo leer ABZ_RES_LOW para no pisar bits de ABZ_RES ni ABZ_SWAP
    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = ABZ_RES_LOW,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer ABZ_RES_LOW: %s", esp_err_to_name(error));
        return error;
    }

    abzResLow = operacion.rx_data[0];
    
    // Una vez leido el registro, lo modifico
    // Se toma que cualquier numero > 0 en abzOff pondra en 1 el bit del registro
    abzResLow = (abzOff > 0) ? (abzResLow | 0x02) : (abzResLow & 0xFD);

    operacion.cmd = WRITE;
    
    operacion.tx_data[0] = abzResLow;

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al escribir ABZ_RES_LOW: %s", esp_err_to_name(error));
        return error;
    }

    printf("Escrito ABZ_OFF: %d\n", abzOff);

    return ESP_OK;
}

esp_err_t mt6835_get_abz_swap(spi_device_handle_t *mt6835Handle, uint8_t *abzSwap) {
    esp_err_t error;

    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = ABZ_RES_LOW,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer ABZ_SWAP: %s", esp_err_to_name(error));
        return error;
    }

    *abzSwap = operacion.rx_data[0] & 0x01;

    printf("Leido ABZ_SWAP: %d\n", *abzSwap);

    return ESP_OK;
}

esp_err_t mt6835_set_abz_swap(spi_device_handle_t *mt6835Handle, uint8_t abzSwap) {
    esp_err_t error;
    uint8_t abzResLow = 0;

    // Primero debo leer ABZ_RES_LOW para no pisar bits de ABZ_RES ni ABZ_OFF
    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = ABZ_RES_LOW,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer ABZ_RES_LOW: %s", esp_err_to_name(error));
        return error;
    }

    abzResLow = operacion.rx_data[0];
    
    // Una vez leido el registro, lo modifico
    // Se toma que cualquier numero > 0 en abzSwap pondra en 1 el bit del registro
    abzResLow = (abzSwap > 0) ? (abzResLow | 0x01) : (abzResLow & 0xFE);

    operacion.cmd = WRITE;
    
    operacion.tx_data[0] = abzResLow;

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al escribir ABZ_RES_LOW: %s", esp_err_to_name(error));
        return error;
    }

    printf("Escrito ABZ_SWAP: %d\n", abzSwap);

    return ESP_OK;
}

esp_err_t mt6835_set_cur_position_zero(spi_device_handle_t *mt6835Handle) {
    esp_err_t error;

    spi_transaction_t operacion = {
        .cmd = SET_ZERO,
        .addr = 0x000,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK || operacion.rx_data[0] != 0x55) {
        ESP_LOGE(tag, "Problema al realizar cero en MT6835: %s", esp_err_to_name(error));
        ESP_LOGE(tag, "ACK: 0x%02X != 0x55", operacion.rx_data[0]);
        return error;
    }

    printf("Cero realizado\n");

    return ESP_OK;
}

esp_err_t mt6835_set_zero(spi_device_handle_t *mt6835Handle, float angle) {
    if (angle < 0.0 || angle > 360.0) {
        ESP_LOGW(tag, "El angulo debe estar entre 0° y 360°.");
        return ESP_FAIL;
    }

    esp_err_t error;

    uint8_t zEdge = 0, zWidth = 0;

    if (mt6835_get_z_edge(mt6835Handle, &zEdge) != ESP_OK) {
        return ESP_FAIL;
    }

    if (mt6835_get_z_pulse_width(mt6835Handle, &zWidth) != ESP_OK) {
        return ESP_FAIL;
    }

    uint16_t temp = angle * 4095 / 360;
    printf("temp: %d\n", temp);

    spi_transaction_t operacion = {
        .cmd = WRITE,
        .addr = ZERO_HIGH,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    // ZERO_POS es de 12 bits, guardo los primeros 8 en el high byte
    operacion.tx_data[0] = temp >> 4;

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al escribir ZERO_HIGH: %s", esp_err_to_name(error));
        return error;
    }

    // Bitshift para bits de z edge y z pulse width
    temp <<= 4;
    // Bit de z edge
    temp = (zEdge > 0) ? (temp | 0x08) : (temp & 0xF7);
    // Bits de z pulse width
    temp |= zWidth;

    operacion.addr = ZERO_LOW;

    operacion.tx_data[0] = temp;

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al escribir ZERO_LOW: %s", esp_err_to_name(error));
        return error;
    }

    printf("Cero seteado en: %f\n", temp * 0.088);

    return ESP_OK;
    
}

esp_err_t mt6835_get_z_edge(spi_device_handle_t *mt6835Handle, uint8_t *zEdge) {
    esp_err_t error;

    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = ZERO_LOW,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer ZERO_LOW: %s", esp_err_to_name(error));
        return error;
    }

    *zEdge = (operacion.rx_data[0] & 0x08) >> 3;

    printf("Z_EDGE: %d\n0: Flanco de subida alineado con 0°\n1: Flanco de bajada alineado con 0°", *zEdge);

    return ESP_OK;
}

esp_err_t mt6835_set_z_edge(spi_device_handle_t *mt6835Handle, uint8_t zEdge) {
    esp_err_t error;

    uint8_t temp = 0;

    // Primero leo ZERO_LOW para no pisar bits de ZERO_POS y Z_WIDTH
    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = ZERO_LOW,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer ZERO_LOW: %s", esp_err_to_name(error));
        return error;
    }

    temp = operacion.rx_data[0];

    // Borro bit de z edge
    temp &= 0xF7;
    // Pongo bit en 1 para todo zEdge > 0
    temp = (zEdge > 0) ? (temp | 0x08) : (temp);

    operacion.cmd = WRITE;

    operacion.tx_data[0] = temp;

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al escribir ZERO_LOW: %s", esp_err_to_name(error));
        return error;
    }

    printf("Z_EDGE seteado: %d\n0: Flanco de subida alineado con 0°\n1: Flanco de bajada alineado con 0°", (temp & 0x08) >> 3);

    return ESP_OK;
}

esp_err_t mt6835_get_z_pulse_width(spi_device_handle_t *mt6835Handle, uint8_t *zWidth) {
    esp_err_t error;

    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = ZERO_LOW,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer ZERO_LOW: %s", esp_err_to_name(error));
        return error;
    }

    *zWidth = operacion.rx_data[0] & 0x07;

    switch (*zWidth) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
            printf("Ancho de pulso de Z: %d LSB\n", 1 << *zWidth);
            break;
        case 5:
            printf("Ancho de pulso de Z: 60°\n");
            break;
        case 6:
            printf("Ancho de pulso de Z: 120°\n");
            break;
        case 7:
            printf("Ancho de pulso de Z: 180°\n");
            break;
        default:
            break;
    }

    return ESP_OK;
}

esp_err_t mt6835_set_z_pulse_width(spi_device_handle_t *mt6835Handle, uint8_t zWidth) {
    if (zWidth > 0x07) {
        ESP_LOGW(tag, "El valor del registro Z_WIDTH debe estar entre 0 y 7");
        return ESP_FAIL;
    }

    esp_err_t error;

    uint8_t temp = 0;

    // Primero leo ZERO_LOW para no pisar bits de ZERO_POS y Z_EDGE
    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = ZERO_LOW,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer ZERO_LOW: %s", esp_err_to_name(error));
        return error;
    }

    temp = operacion.rx_data[0];

    // Borro bits de z width
    temp &= 0xF8;
    // Como los bits estan en 0 solo hago un OR
    temp |= zWidth;

    operacion.cmd = WRITE;

    operacion.tx_data[0] = temp;

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al escribir ZERO_LOW: %s", esp_err_to_name(error));
        return error;
    }

    printf("Z_WIDTH seteado: %d\n", (temp & 0x08) >> 3);

    return ESP_OK;
}

esp_err_t mt6835_get_z_phase(spi_device_handle_t *mt6835Handle, uint8_t *zPhase) {
    esp_err_t error;

    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = UVW_CONF,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer UVW_CONF: %s", esp_err_to_name(error));
        return error;
    }

    *zPhase = (operacion.rx_data[0] & 0xC0) >> 6;

    printf("Z_PHASE: %d\n", *zPhase);

    return ESP_OK;
}

esp_err_t mt6835_set_z_phase(spi_device_handle_t *mt6835Handle, uint8_t zPhase) {
    if (zPhase > 0x03) {
        ESP_LOGW(tag, "El valor del registro Z_PHASE debe estar entre 0 y 3");
        return ESP_FAIL;
    }

    esp_err_t error;

    uint8_t temp;

    // Primero leo UVW_CONF para no pisar bits UVW_MUX, UVW_OFF y UVW_RES
    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = UVW_CONF,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer UVW_CONF: %s", esp_err_to_name(error));
        return error;
    }

    temp = operacion.rx_data[0];

    // Borro bits de Z_PHASE
    temp &= 0x3F;
    // Como los bits estan en 0 solo hago un OR, pero debo bitshiftear zPhase para alinear
    temp |= (zPhase << 6);

    operacion.cmd = WRITE;

    operacion.tx_data[0] = temp;

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al escribir UVW_CONF: %s", esp_err_to_name(error));
        return error;
    }

    printf("Z_PHASE grabado: %d\n", zPhase);

    return ESP_OK;
}

esp_err_t mt6835_get_abz_lead(spi_device_handle_t *mt6835Handle, uint8_t *abLead) {
    esp_err_t error;

    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = HYST,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer HYST: %s", esp_err_to_name(error));
        return error;
    }

    *abLead = (operacion.rx_data[0] & 0x08) >> 3;

    switch (*abLead) {
        case CCW_BA:
            printf("B adelanta A en giro antihorario\n");
            break;
        case CCW_AB:
            printf("A adelanta B en giro antihorario\n");
            break;
        default:
            printf("Registo mal leido\n");
            return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t mt6835_set_abz_lead(spi_device_handle_t *mt6835Handle, uint8_t abLead) {
    esp_err_t error;

    uint8_t temp = 0;

    // Primero leo registro para no pisar HYST y parte prioritaria
    spi_transaction_t operacion = {
        .cmd = READ,
        .addr = HYST,
        .length = 24,
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA
    };

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al leer HYST: %s", esp_err_to_name(error));
        return error;
    }

    temp = operacion.rx_data[0];

    // Pongo en 0 el bit de ROT_DIR
    temp &= 0xF7;
    // Si abLead > 0, pongo el bit en 1 (CCW_AB)
    temp = (abLead > 0) ? (temp | CCW_AB) : (temp);

    operacion.cmd = WRITE;

    operacion.tx_data[0] = temp;

    error = spi_device_transmit(*mt6835Handle, &operacion);

    if (error != ESP_OK) {
        ESP_LOGE(tag, "Error al escribir HYST: %s", esp_err_to_name(error));
        return error;
    }

    switch (abLead) {
        case CCW_BA:
            printf("B adelanta A en giro antihorario\n");
            break;
        case CCW_AB:
            printf("A adelanta B en giro antihorario\n");
            break;
        default:
            return ESP_FAIL;
    }

    return ESP_OK;
}



