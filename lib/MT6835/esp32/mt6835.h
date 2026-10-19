#ifndef MT6835_H
#define MT6835_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

typedef enum MT6835_CMD_t {
    READ        = 0b0011,
    WRITE       = 0b0110,
    PROG_EEPROM = 0b1100,
    SET_ZERO    = 0b0101,
    BURST_READ  = 0b1010
};

typedef enum MT6835_ADDR_t {
    USER_ID      = 0x001,
    ANGLE_HIGH   = 0x003,
    ANGLE_MID    = 0x004,
    ANGLE_LOW    = 0x005,   // 0:2 STATUS - 3:7 ANGLE1
    CRC          = 0x006,
    ABZ_RES_HIGH = 0x007,
    ABZ_RES_LOW  = 0x008,   // 0 Swap AB - 1 ABZ Off - 2:7 ABZ_RES1
    ZERO_HIGH    = 0x009,
    ZERO_LOW     = 0x00A,   // 0:2 Z pulse width - 3 Z edge - 4:7 ZERO1
    UVW_CONF     = 0x00B,   // 0:3 UVW_RES - 4 UVW_OFF - 5 UVW_MUX - 6:7 Z_PHASE
    PWM_CONF     = 0x00C,   // 0:2 PWM_SEL - 3 PWM_POL - 4 PWM_FQ - 5 NLC_EN
    HYST         = 0x00D,   // 0:2 HYST - 3 ROT_DIR
    AUTOCAL_FREQ = 0x00E,   // 4:6 AUTOCAL_FREQ - 7 GPIO_DS
    BW           = 0x011,  // 0:2 BW
    NLC_START    = 0x013,
    NLC_END      = 0x0D2
};

typedef enum MT6835_ROT_DIR_t {
    CCW_BA = 0b00000000,
    CCW_AB = 0b00001000
};

esp_err_t mt6835_get_user_id(spi_device_handle_t *mt6835Handle, uint8_t *userID);
esp_err_t mt6835_set_user_id(spi_device_handle_t *mt6835Handle, uint8_t userID);
esp_err_t mt6835_get_angle(spi_device_handle_t *mt6835Handle, uint32_t *angle);
uint8_t calculate_crc(uint32_t angle);
esp_err_t mt6835_program_eeprom(spi_device_handle_t *mt6835Handle);
esp_err_t mt6835_get_abz_res(spi_device_handle_t *mt6835Handle, uint16_t *abzRes);
esp_err_t mt6835_set_abz_res(spi_device_handle_t *mt6835Handle, uint16_t abzRes);
esp_err_t mt6835_get_abz_off(spi_device_handle_t *mt6835Handle, uint8_t *abzOff);
esp_err_t mt6835_set_abz_off(spi_device_handle_t *mt6835Handle, uint8_t abzOff);
esp_err_t mt6835_get_abz_swap(spi_device_handle_t *mt6835Handle, uint8_t *abzSwap);
esp_err_t mt6835_set_abz_swap(spi_device_handle_t *mt6835Handle, uint8_t abzSwap);
esp_err_t mt6835_set_cur_position_zero(spi_device_handle_t *mt6835Handle);          // Posicion actual como 0
esp_err_t mt6835_set_zero(spi_device_handle_t *mt6835Handle, float angle);          // Se pasa la posicion que se desea como 0
esp_err_t mt6835_get_z_edge(spi_device_handle_t *mt6835Handle, uint8_t *zEdge);
esp_err_t mt6835_set_z_edge(spi_device_handle_t *mt6835Handle, uint8_t zEdge);
esp_err_t mt6835_get_z_pulse_width(spi_device_handle_t *mt6835Handle, uint8_t *zWidth);
esp_err_t mt6835_set_z_pulse_width(spi_device_handle_t *mt6835Handle, uint8_t zWidth);
esp_err_t mt6835_get_z_phase(spi_device_handle_t *mt6835Handle, uint8_t *zPhase);
esp_err_t mt6835_set_z_phase(spi_device_handle_t *mt6835Handle, uint8_t zPhase);
esp_err_t mt6835_get_abz_lead(spi_device_handle_t *mt6835Handle, uint8_t *abLead);
esp_err_t mt6835_set_abz_lead(spi_device_handle_t *mt6835Handle, uint8_t abLead);


#endif