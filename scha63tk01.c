/*
 * scha63tk01.c
 *
 *  Created on: Feb 13, 2024
 *      Author: Plan-S
 */

#include "config.h"
#ifdef INCLUDE_SCHA63TK01
#include <string.h>
#include <stdlib.h>
#include "../Inc/scha63tk01.h"
#include "../Inc/scha63x.h"
#include "gpio.h"
#include "spi.h"

#define prv_log_msg(msgLogLevel, ...) logMessage(__FUNCTION__, logDEBUG, msgLogLevel, __VA_ARGS__)

#define RAD_TO_DEG                             (57.2957795131f)
#define SCHA63TK01_TEMP_SCALE                  (0.0078125f)

#define SCHA63TK01_DATA_CHECK_RS_ERROR(a)      ((((a) >> 24) & 0x03) != 1 ? TRUE : FALSE) // true = RS error
#define SCHA63TK01_DATA_INT8_UPPER(a)          ((int8_t)(((a) >> 16) & 0xff))
#define SCHA63TK01_DATA_INT8_LOWER(a)          ((int8_t)(((a) >> 8) & 0xff))
#define SCHA63TK01_DATA_INT16(a)               ((int16_t)(((a) >> 8) & 0xffff))
#define SCHA63TK01_DATA_UINT16(a)              ((uint16_t)(((a) >> 8) & 0xffff))

#define REG_DUE_RZ   0x01     // SCHA63T register DUE Rate Z
#define REG_DUE_RY   0x03     // SCHA63T register UNO Rate Y
#define REG_DUE_RZ2  0x0B     // SCHA63T register UNO Rate Z2
#define REG_DUE_RY2  0x0D     // SCHA63T register UNO Rate Y2
#define REG_UNO_RX   0x01     // SCHA63T register DUE Rate X
#define REG_UNO_AX   0x04     // SCHA63T register DUE Acc X
#define REG_UNO_AY   0x05     // SCHA63T register DUE Acc Y
#define REG_UNO_AZ   0x06     // SCHA63T register DUE Acc Z
#define REG_UNO_RX2  0x0B     // SCHA63T register DUE Rate X2
#define REG_TEMP     0x07     // SCHA63T register temperature
#define REG_SUMSTAT  0x0E     // SCHA63T register summary status
#define REG_FILT_G   0x16     // SCHA63T register gyro filter control
#define REG_SYS_TEST 0x17     // SCHA63T register SYS_TEST
#define REG_RESET    0x18     // SCHA63T register reset control
#define REG_MODE     0x19     // SCHA63T register mode control
#define REG_FILT_A   0x1A     // SCHA63T register ACC filter control
#define REG_CID      0x1B     // SCHA63T register component ID
#define REG_TR2      0x1C     // SCHA63T register traceability 2
#define REG_TR0      0x1D     // SCHA63T register traceability 0
#define REG_TR1      0x1E     // SCHA63T register traceability 1
#define REG_SELBNK   0x1F     // SCHA63T register select bank
#define SCLK_RATE    10000000 // SCHA63T SPI SCLK rate (Hz)

#define BANK_0       0x00     // SCHA63T bank 0
#define BANK_1       0x01     // SCHA63T bank 1
#define BANK_2       0x02     // SCHA63T bank 2
#define BANK_3       0x03     // SCHA63T bank 3
#define BANK_4       0x04     // SCHA63T bank 4
#define BANK_5       0x05     // SCHA63T bank 5

#define SCALE_GYRO (1.0/80)   // SCHA63T gyro scale (deg/s/LSB) (nominal)
#define SCALE_ACCL (1.0/4905) // SCHA63T accelerometer scale (g/LSB) (nominal)
#define SCALE_TEMP (1.0/30)   // SCHA63T temperature scale (C/LSB)


extern scha63x_cacv scha63x_cac_values;

typedef enum _scha63tk01_op_read_write
{
    SCHA63TK01_OP_READ = 0,
    SCHA63TK01_OP_WRITE
} scha63tk01_op_read_write_t;

typedef enum _scha63tk01_asic_id
{
    SCHA63TK01_ASIC_DUE = 0,
    SCHA63TK01_ASIC_UNO
} scha63tk01_asic_id_t;

static status_t scha63tk01_read_serial_number
    (
    scha63tk01_handle_t *p_handle,
    char *p_serial_number
    );
static uint8_t scha63tk01_check_init
    (
    scha63tk01_handle_t *p_handle,
    uint8_t asic_id
    );
static status_t scha63tk01_test
    (
    scha63tk01_handle_t *p_handle
    );
static status_t scha63tk01_transaction
    (
    scha63tk01_handle_t *p_handle,
    uint8_t asic_id,
    uint8_t op_read_write,
    uint8_t op_reg_addr,
    uint16_t write_data,
    uint32_t *p_read_buffer
    );
static uint8_t scha63tk01_crc8 (uint8_t bit_value, uint8_t crc);

extern uint32_t SPI_ASIC_DUE(uint32_t dout);
extern uint32_t SPI_ASIC_UNO(uint32_t dout);

typedef struct _scha63tk01_ram_data
{
    scha63tk01_handle_t *p_handle;
} scha63tk01_ram_data_t;

scha63tk01_ram_data_t s_scha63tk01_ram_data;

status_t scha63tk01_init (scha63tk01_handle_t *p_handle, scha63tk01_config_t *p_config)
{
    status_t status = OK;
    status_t ret_val = OK;
    uint32_t buffer_read = 0;
    uint8_t status_uno = FALSE;
    uint8_t status_due = FALSE;
    uint8_t attempt = 0;
    uint8_t num_attempts = 2;
    uint16_t write_buffer_g_filt = 0;
    uint16_t write_buffer_a_filt = 0;

    for (;;)
    {
        if (NULL == p_handle || NULL == p_config)
        {
            status = ERRNO(EINVAL);
            break;
        }

        memcpy(&p_handle->config, p_config, sizeof(scha63tk01_config_t));
        s_scha63tk01_ram_data.p_handle = p_handle;

        write_buffer_g_filt = (((p_config->filter_config.gyro_filter_y & 0x7) << 0) | ((p_config->filter_config.gyro_filter_zx & 0x7) << 8));
        write_buffer_a_filt = (((p_config->filter_config.acc_filter[2] & 0x7) << 0) | ((p_config->filter_config.acc_filter[1] & 0x7) << 4) | ((p_config->filter_config.acc_filter[1] & 0x7) << 8));

        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_WRITE, REG_RESET, 0x0001, NULL);
        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_SELBNK, BANK_0, NULL);
        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_RESET, 0x0001, NULL);
        osDelay(25);

        status = scha63tk01_read_serial_number(p_handle, p_handle->serial_num);
        if (status != OK)
        {
            break;
        }

        status = scha63tk01_test(p_handle);
        if (OK != status)
        {
            break;
        }

        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_SELBNK, BANK_0, NULL);
        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_RESET, 0x0001, NULL);
        osDelay(25);

        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_WRITE, REG_MODE, 0, NULL);
        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_MODE, 0, NULL);
        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_MODE, 0, NULL);
        osDelay(70);


        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_WRITE, REG_FILT_G, write_buffer_g_filt, NULL);
        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_WRITE, REG_FILT_A, write_buffer_a_filt, NULL);

        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_RESET, 0x0001, NULL);
        osDelay(25);

        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_MODE, 0, NULL);
        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_MODE, 0, NULL);
        osDelay(1);

        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_FILT_G, write_buffer_g_filt, NULL);

        for (attempt = 0; attempt < num_attempts; attempt++)
        {
            osDelay(405);

            scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_WRITE, REG_RESET, 0x0002, NULL);
            scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_RESET, 0x0002, NULL);

            status_uno = scha63tk01_check_init(p_handle, SCHA63TK01_ASIC_UNO);
            status_due = scha63tk01_check_init(p_handle, SCHA63TK01_ASIC_DUE);

            if ((status_uno == FALSE || status_due == FALSE) && attempt < (num_attempts - 1))
            {
                scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_WRITE, REG_RESET, 0x0001, NULL);
                scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_RESET, 0x0001, NULL);
                osDelay(25);
                scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_WRITE, REG_MODE, 0, NULL);
                scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_MODE, 0, NULL);
                scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_MODE, 0, NULL);
                osDelay(50);
                scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_WRITE, REG_FILT_G, write_buffer_g_filt, NULL);
                scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_WRITE, REG_FILT_A, write_buffer_a_filt, NULL);
                scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_DUE, SCHA63TK01_OP_WRITE, REG_FILT_G, write_buffer_g_filt, NULL);
                osDelay(45);
            }
            else
            {
                break;
            }
        }
        if (status_uno == FALSE || status_due == FALSE)
        {
            status = ERRNO(EFAILED);
            break;
        }

        break;
    }

    return status;
}

status_t scha63tk01_read_serial_number (scha63tk01_handle_t *p_handle, char *p_serial_number)
{
    status_t status = OK;
    uint16_t data_to_read = 0;
    uint16_t data_to_write = 0;
    uint32_t trc_0 = 0;
    uint32_t trc_1 = 0;
    uint32_t trc_2 = 0;
    uint16_t id_0 = 0;
    uint16_t id_1 = 0;
    uint16_t id_2 = 0;

    for (;;)
    {
        if (NULL == p_handle)
        {
            status = ERRNO(EINVAL);
            break;
        }

        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_READ, REG_TR2, 0, NULL);
        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_READ, REG_TR0, 0, &trc_2);
        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_READ, REG_TR1, 0, &trc_0);
        scha63tk01_transaction(p_handle, SCHA63TK01_ASIC_UNO, SCHA63TK01_OP_READ, REG_TR1, 0, &trc_1);

        trc_2 = SCHA63TK01_DATA_UINT16(trc_2);
        trc_0 = SCHA63TK01_DATA_UINT16(trc_0);
        trc_1 = SCHA63TK01_DATA_UINT16(trc_1);

        id_1 = (trc_2 >> 8) & 0x0F;
        id_0 = (trc_0) & 0xFFFF;
        id_2 = (trc_1) & 0xFFFF;

        snprintf(p_serial_number, 14, "%05d%01x%04X", id_2, id_1, id_0);
        break;
    }

    return status;
}

uint8_t scha63tk01_check_init
    (
    scha63tk01_handle_t *p_handle,
    uint8_t asic_id
    )
{
    uint8_t status = FALSE;
    uint32_t read_buffer = 0;

    for (;;)
    {
        if (NULL == p_handle)
        {
            break;
        }

        // read summary status two times (first time may show incorrectly FAIL after start-up)
        scha63tk01_transaction(p_handle, asic_id, SCHA63TK01_OP_READ, REG_SUMSTAT, 0, NULL);
        scha63tk01_transaction(p_handle, asic_id, SCHA63TK01_OP_READ, REG_SUMSTAT, 0, NULL);
        osDelay(3);
        scha63tk01_transaction(p_handle, asic_id, SCHA63TK01_OP_READ, REG_SUMSTAT, 0, &read_buffer);
        status = SCHA63TK01_DATA_CHECK_RS_ERROR(read_buffer) == TRUE ? FALSE : TRUE;

        break;
    }

    return status;
}

status_t scha63tk01_test
    (
    scha63tk01_handle_t *p_handle
    )
{
    status_t status = OK;
    uint32_t read_buffer = 0;

    for (;;)
    {
        if (NULL == p_handle)
        {
            status = ERRNO(EINVAL);
            break;
        }

        SPI_ASIC_DUE(0xE40010AA);
        SPI_ASIC_DUE(0x640000A7);
        SPI_ASIC_DUE(0xE400088F);
        SPI_ASIC_DUE(0x640000A7);
        SPI_ASIC_DUE(0xE40020E0);
        SPI_ASIC_DUE(0x640000A7);
        uint32_t resp = SPI_ASIC_DUE(0x640000A7);

        // check if device is correctly set to test mode
        if ((SCHA63TK01_DATA_UINT16(resp) & 0x7) == 7)
        {
            // read cross-axis compensation values
            SPI_ASIC_DUE(0xFC00051A);
            SPI_ASIC_DUE(0x2C0000CB);
            uint32_t cxx_cxy = SPI_ASIC_DUE(0x4C00009B);
            uint32_t cxz_cyx = SPI_ASIC_DUE(0x50000089);
            uint32_t cyy_cyz = SPI_ASIC_DUE(0x5400008F);
            uint32_t czx_czy = SPI_ASIC_DUE(0x58000085);
            uint32_t czz_bxx = SPI_ASIC_DUE(0x5C000083);
            uint32_t bxy_bxz = SPI_ASIC_DUE(0x600000A1);
            uint32_t byx_byy = SPI_ASIC_DUE(0x6C0000AB);
            uint32_t byz_bzx = SPI_ASIC_DUE(0x700000B9);
            uint32_t bzy_bzz = SPI_ASIC_DUE(0x700000B9);

            p_handle->scha63x_cac_values.cxx = SCHA63TK01_DATA_INT8_LOWER(cxx_cxy) / 4096.0 + 1;
            p_handle->scha63x_cac_values.cxy = SCHA63TK01_DATA_INT8_UPPER(cxx_cxy) / 4096.0;
            p_handle->scha63x_cac_values.cxz = SCHA63TK01_DATA_INT8_LOWER(cxz_cyx) / 4096.0;
            p_handle->scha63x_cac_values.cyx = SCHA63TK01_DATA_INT8_UPPER(cxz_cyx) / 4096.0;
            p_handle->scha63x_cac_values.cyy = SCHA63TK01_DATA_INT8_LOWER(cyy_cyz) / 4096.0 + 1;
            p_handle->scha63x_cac_values.cyz = SCHA63TK01_DATA_INT8_UPPER(cyy_cyz) / 4096.0;
            p_handle->scha63x_cac_values.czx = SCHA63TK01_DATA_INT8_LOWER(czx_czy) / 4096.0;
            p_handle->scha63x_cac_values.czy = SCHA63TK01_DATA_INT8_UPPER(czx_czy) / 4096.0;
            p_handle->scha63x_cac_values.czz = SCHA63TK01_DATA_INT8_LOWER(czz_bxx) / 4096.0 + 1;
            p_handle->scha63x_cac_values.bxx = SCHA63TK01_DATA_INT8_UPPER(czz_bxx) / 4096.0 + 1;
            p_handle->scha63x_cac_values.bxy = SCHA63TK01_DATA_INT8_LOWER(bxy_bxz) / 4096.0;
            p_handle->scha63x_cac_values.bxz = SCHA63TK01_DATA_INT8_UPPER(bxy_bxz) / 4096.0;
            p_handle->scha63x_cac_values.byx = SCHA63TK01_DATA_INT8_LOWER(byx_byy) / 4096.0;
            p_handle->scha63x_cac_values.byy = SCHA63TK01_DATA_INT8_UPPER(byx_byy) / 4096.0 + 1;
            p_handle->scha63x_cac_values.byz = SCHA63TK01_DATA_INT8_LOWER(byz_bzx) / 4096.0;
            p_handle->scha63x_cac_values.bzx = SCHA63TK01_DATA_INT8_UPPER(byz_bzx) / 4096.0;
            p_handle->scha63x_cac_values.bzy = SCHA63TK01_DATA_INT8_LOWER(bzy_bzz) / 4096.0;
            p_handle->scha63x_cac_values.bzz = SCHA63TK01_DATA_INT8_UPPER(bzy_bzz) / 4096.0 + 1;

            memcpy(&scha63x_cac_values, &p_handle->scha63x_cac_values, sizeof(scha63x_cac_values));
        }
        else
        {
            // return error if test mode can not be activated
            status =  ERRNO(EFAILED);
            break;
        }

        break;
    }

    return status;
}

status_t scha63tk01_transaction
    (
    scha63tk01_handle_t *p_handle,
    uint8_t asic_id,
    uint8_t op_read_write,
    uint8_t op_reg_addr,
    uint16_t write_data,
    uint32_t *p_read_buffer
    )
{
    status_t status = OK;
    uint32_t cmd = 0;
    uint8_t crc = 0;
    uint8_t bit_value = 0;
    uint32_t ret_val = 0;

    for (;;)
    {
        if (NULL == p_handle)
        {
            status = ERRNO(EINVAL);
            break;
        }

        cmd |= ((op_read_write & 0x03  ) << 31);
        cmd |= ((op_reg_addr   & 0x1F  ) << 26);
        cmd |= ((write_data & 0xFFFF) << 8);

        crc = 0xFF;
        for (uint8_t bit_index = 31; bit_index > 7; bit_index--)
        {
            bit_value = (uint8_t)((cmd >> bit_index) & 0x01);
            crc = scha63tk01_crc8(bit_value, crc);
        }
        crc = (uint8_t)(~crc);

        cmd |= ((crc & 0xFF) << 0);

        switch (asic_id)
        {
        case SCHA63TK01_ASIC_DUE:
            ret_val = SPI_ASIC_DUE(cmd);
            break;

        case SCHA63TK01_ASIC_UNO:
            ret_val = SPI_ASIC_UNO(cmd);
            break;

        default:
            status = ERRNO(EINVAL);
            break;
        }

        // TODO check if the read data is valid

        if (NULL != p_read_buffer)
        {
            *p_read_buffer = ret_val;
        }

        break;
    }

    return status;
}

uint8_t scha63tk01_crc8 (uint8_t bit_value, uint8_t crc)
{
    uint8_t temp = 0;

    temp = (crc & 0x80);

    if (bit_value == 0x01)
    {
        temp ^= 0x80;
    }

    crc <<= 1;

    if (temp > 0)
    {
        crc ^= 0x1D;
    }

    return crc;
}

status_t scha63tk01_get_sensor_data (scha63tk01_handle_t *p_handle, scha63tk01_sensor_data_t *p_data)
{
    status_t status = OK;
    scha63x_real_data scha63x_data;
    scha63x_raw_data scha63x_raw_data_last;
    scha63x_raw_data_summed scha63x_data_summed_raw[1];

    for (;;)
    {
        if ((NULL == p_handle) || (NULL == p_data))
        {
            status = ERRNO(EINVAL);
            break;
        }

        memset(&scha63x_data, 0, sizeof(scha63x_data));
        memset(&scha63x_raw_data_last, 0, sizeof(scha63x_raw_data_last));
        memset(&scha63x_data_summed_raw, 0, sizeof(scha63x_data_summed_raw));

        scha63x_read_data(&scha63x_raw_data_last);

        scha63x_data_summed_raw[0].acc_x_lsb += scha63x_raw_data_last.acc_x_lsb;
        scha63x_data_summed_raw[0].acc_y_lsb += scha63x_raw_data_last.acc_y_lsb;
        scha63x_data_summed_raw[0].acc_z_lsb += scha63x_raw_data_last.acc_z_lsb;
        scha63x_data_summed_raw[0].gyro_x_lsb += scha63x_raw_data_last.gyro_x_lsb;
        scha63x_data_summed_raw[0].gyro_y_lsb += scha63x_raw_data_last.gyro_y_lsb;
        scha63x_data_summed_raw[0].gyro_z_lsb += scha63x_raw_data_last.gyro_z_lsb;
        scha63x_data_summed_raw[0].temp_due_lsb += scha63x_raw_data_last.temp_due_lsb;
        scha63x_data_summed_raw[0].temp_uno_lsb += scha63x_raw_data_last.temp_uno_lsb;

        scha63x_convert_data(&scha63x_data_summed_raw[0], &scha63x_data);

        scha63x_cross_axis_compensation(&scha63x_data);

        p_data->gyro.x = scha63x_data.gyro_x * RAD_TO_DEG / 2.0;
        p_data->gyro.y = scha63x_data.gyro_y * RAD_TO_DEG / 2.0;
        p_data->gyro.z = -1 * scha63x_data.gyro_z * RAD_TO_DEG / 2.0; // multiple by -1 since the sensor is left handed
        p_data->temperature = (scha63x_data.temp_due + scha63x_data.temp_uno) / 2.0;

        break;
    }

    return status;
}

status_t scha63tk01_dump (scha63tk01_handle_t *p_handle)
{
    status_t status = OK;

    for (;;)
    {
        break;
    }

    return status;
}

/*----------------------------------------------------------------------------
  SPI communication
 *----------------------------------------------------------------------------*/
static uint32_t SPI_ASIC(void* spi_handle, uint32_t dout)
{
    uint32_t resp = 0;
    uint32_t data_out = 0;

    data_out |= (dout & 0x000000ff) << 24;
    data_out |= (dout & 0x0000ff00) << 8;
    data_out |= (dout & 0x00ff0000) >> 8;
    data_out |= (dout & 0xff000000) >> 24;

    HAL_SPI_TransmitReceive(spi_handle, &data_out, &resp, sizeof(data_out), 1000);

    resp = ((resp & 0x000000ff)) << 24 | ((resp & 0x0000ff00)) << 8 | ((resp & 0x00ff0000)) >> 8 | ((resp & 0xff000000)) >> 24;

    return resp;
}

/*----------------------------------------------------------------------------
  SPI communication to ASIC1 (DUE) using CS1
 *----------------------------------------------------------------------------*/
uint32_t SPI_ASIC_DUE(uint32_t dout)
{
	uint32_t ret_val = 0;
	HAL_GPIO_WritePin(s_scha63tk01_ram_data.p_handle->config.due.cs_pin_port, s_scha63tk01_ram_data.p_handle->config.due.cs_pin, GPIO_PIN_RESET);
	ret_val = SPI_ASIC(s_scha63tk01_ram_data.p_handle->config.due.p_spi_handle, dout);
	HAL_GPIO_WritePin(s_scha63tk01_ram_data.p_handle->config.due.cs_pin_port, s_scha63tk01_ram_data.p_handle->config.due.cs_pin, GPIO_PIN_SET);
	return ret_val;
}


/*----------------------------------------------------------------------------
  SPI communication to ASIC2 (UNO) using CS2
 *----------------------------------------------------------------------------*/
uint32_t SPI_ASIC_UNO(uint32_t dout)
{
	uint32_t ret_val = 0;
    // TODO reset cs pin
	ret_val = SPI_ASIC(s_scha63tk01_ram_data.p_handle->config.uno.p_spi_handle, dout);
    // TODO set cs pin
	HAL_GPIO_WritePin(s_scha63tk01_ram_data.p_handle->config.due.cs_pin_port, s_scha63tk01_ram_data.p_handle->config.due.cs_pin, GPIO_PIN_SET);
	return ret_val;
}
#endif /* INCLUDE_SCHA63TK01 */
