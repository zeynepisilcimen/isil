/*
 * scha63tk01.h
 *
 *  Created on: August 28, 2024
 *      Author: PlanS
 */

#ifndef INCLUDED_SCHA63TK01_H
#define INCLUDED_SCHA63TK01_H

#include <stdint.h>
#include <errno.h>

typedef int32_t status_t;

#define TRUE                    (0xFF)
#define FALSE                   (0)
#define OK                      (0)
#define ERRNO(x)                (-x)
#define EFAILED                 (1)


typedef enum _scha63tk01_gyro_filter
{
    SCHA63TK01_GYRO_FILTER_13HZ = 0,
    SCHA63TK01_GYRO_FILTER_20HZ,
    SCHA63TK01_GYRO_FILTER_46HZ,
    SCHA63TK01_GYRO_FILTER_200HZ,
    SCHA63TK01_GYRO_FILTER_300HZ,
} scha63tk01_gyro_filter_t;

typedef enum _scha63tk01_acc_filter
{
    SCHA63TK01_ACC_FILTER_13HZ = 0,
    SCHA63TK01_ACC_FILTER_20HZ,
    SCHA63TK01_ACC_FILTER_46HZ,
    SCHA63TK01_ACC_FILTER_200HZ,
    SCHA63TK01_ACC_FILTER_300HZ,
} scha63tk01_acc_filter_t;

typedef struct _scha64tk01_filter_config
{
    scha63tk01_gyro_filter_t gyro_filter_zx;
    scha63tk01_gyro_filter_t gyro_filter_y;
    scha63tk01_acc_filter_t acc_filter[3];
} scha64tk01_filter_config_t;

typedef struct _scha63tk01_config
{
    struct
    {
    	uint16_t cs_pin;
    	void* cs_pin_port;
        void* p_spi_handle;
    } uno;
    struct
    {
    	uint16_t cs_pin;
    	void* cs_pin_port;
        void *p_spi_handle;
    } due;
    scha64tk01_filter_config_t filter_config;
} scha63tk01_config_t;

typedef struct _scha63tk01_axis
{
    float x;
    float y;
    float z;
} scha63tk01_axis_t;

typedef struct _scha63tk01_sensor_data
{
    scha63tk01_axis_t gyro;            /* [dps] */
    float temperature;                 /* [C] */
} scha63tk01_sensor_data_t;

typedef struct _scha63tk01_cacv
{
    float cxx;
    float cxy;
    float cxz;
    float cyx;
    float cyy;
    float cyz;
    float czx;
    float czy;
    float czz;
    float bxx;
    float bxy;
    float bxz;
    float byx;
    float byy;
    float byz;
    float bzx;
    float bzy;
    float bzz;
} scha63tk01_cacv_t;

typedef struct _scha63tk01_handle
{
    scha63tk01_config_t config;
    scha63tk01_cacv_t scha63x_cac_values;
    char serial_num[14];
} scha63tk01_handle_t;

extern status_t scha63tk01_init (scha63tk01_handle_t *p_handle, scha63tk01_config_t *p_config);
extern status_t scha63tk01_get_sensor_data (scha63tk01_handle_t *p_handle, scha63tk01_sensor_data_t *p_data);
extern status_t scha63tk01_dump (scha63tk01_handle_t *p_handle);

#endif /* INCLUDED_SCHA63TK01_H */
