// App-sensorapp-Src-app.c
	// TtODO create task
	// SPI init
	// print acc_x acc_y acc_z gyro_x gyro_y temp

#include "main.h"
#include "usart.h"
#include "spi.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "../Inc/scha63tk01.h"

osThreadId_t myTask04Handle;

/* Definitions for myTask04 */
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


void StartTask04(void *argument)
{
	status_t status = OK;
	scha63tk01_handle_t handle = {};
	scha63tk01_config_t config =


	{
			.due.cs_pin = GPIO_PIN_8, // SPI chip select pin (CS) tanımlaması
			.due.cs_pin_port = GPIOE,// SPI chip select pin'in bağlı olduğu GPIO portu
			.due.p_spi_handle = &hspi1, //SPI iletişimi için kullanılan SPI handle
			.uno.cs_pin = GPIO_PIN_9, // Bir diğer chip select pin (not used)
			.uno.cs_pin_port = GPIOE,  // Bir diğer chip select pin portu (not used)
			.uno.p_spi_handle = &hspi1   // Bir diğer SPI handle (not used)
	};
	// Sensor veri yapısının tanımlanması

	scha63tk01_sensor_data_t sensor_data = {};
	// Sensörün başlatılması

	status = scha63tk01_init(&handle, &config);
	if (OK != status)
	{
		printf("Error occured while initializing driver!");
	}

    for (;;)
    {
    	scha63tk01_get_sensor_data(&handle, &sensor_data);

        printf("%.02f, %.02f, %.02f, %.02f\r\n", sensor_data.gyro.x, sensor_data.gyro.y, sensor_data.gyro.z, sensor_data.temperature);
        osDelay(1000);
}
    }

void appInit(void) {
    /* Create the thread(s) */
    myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);
}


//int taskMain(void) {
//    /* Infinite loop */
//    while (1) {
//        // Gelen bir byte'ı bekleyip alalım
//        uint8_t received;
//        if (HAL_UART_Receive(&huart3, &received, 1, HAL_MAX_DELAY) == HAL_OK) {
//            // Alınan byte'ı tekrar gönderelim (Echo)
//            HAL_UART_Transmit(&huart3, &received, 1, HAL_MAX_DELAY);
//        }
//    }
//}
/* Function to handle printf redirection to UART */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}


















































