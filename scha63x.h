#ifndef SCHA63X_H
#define SCHA63X_H

#include <stdint.h>
#include <stdbool.h>


// With 2.6 kHz sample rate, this produces 100 Hz Output Data Rate (ODR)
#define SAMPLE_COUNT 26


// Raw data values from sensor
typedef struct _scha63x_raw_data {

    int16_t acc_x_lsb;
    int16_t acc_y_lsb;
    int16_t acc_z_lsb;
    int16_t gyro_x_lsb;
    int16_t gyro_y_lsb;
    int16_t gyro_z_lsb;
    int16_t temp_due_lsb;
    int16_t temp_uno_lsb;

    bool rs_error_due;
    bool rs_error_uno;

} scha63x_raw_data;


// Summed raw data values from sensor
typedef struct _scha63x_raw_data_summed {

    int32_t acc_x_lsb;
    int32_t acc_y_lsb;
    int32_t acc_z_lsb;
    int32_t gyro_x_lsb;
    int32_t gyro_y_lsb;
    int32_t gyro_z_lsb;
    int32_t temp_due_lsb;
    int32_t temp_uno_lsb;

} scha63x_raw_data_summed;


// Converted (real) data from sensor
// size = 32 bytes / element
typedef struct _scha63x_real_data {

    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp_due;
    float temp_uno;

} scha63x_real_data;


// Cross axis compensation values
typedef struct _scha63x_cacv {

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

} scha63x_cacv;


// Sensor status
typedef struct _scha63x_sensor_status {

    uint16_t summary_status;
    uint16_t rate_status1;
    uint16_t rate_status2;
    uint16_t acc_status1;
    uint16_t common_status1;
    uint16_t common_status2;

} scha63x_sensor_status;


// SCHA63X_OK library return codes
// Negative values = errors, positive values = warnings
// NOTE: Error code always overrides warning, when returning from function

#define SCHA63X_OK                         0
#define SCHA63X_ERR_TEST_MODE_ACTIVATION   -1 // Could not activate test mode during init
#define SCHA63X_ERR_RS_STATUS_NOK          -2 // RS status not OK after all init steps

extern int  scha63x_init(char *serial_num);
extern void scha63x_read_data(scha63x_raw_data *data);

extern void scha63x_convert_data(scha63x_raw_data_summed *data_in, scha63x_real_data *data_out);
extern void scha63x_cross_axis_compensation(scha63x_real_data *data);

extern void scha63x_read_sensor_status_uno(scha63x_sensor_status *status);
extern void scha63x_read_sensor_status_due(scha63x_sensor_status *status);

#endif // #ifndef SCHA63X_H
