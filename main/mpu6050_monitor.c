#include <stdio.h>
#include "driver/i2c.h"
#include "mpu6050.h"
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

// for esp32 almost any gpio can be used for i2c
#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static mpu6050_handle_t mpu6050 = NULL;

static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK)
    {
        printf("Could not set i2c config\n");
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        printf("Could not install i2c driver\n");
    }
}

static void i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret;

    i2c_bus_init();
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    if (mpu6050 == NULL)
    {
        printf("Create mpu6050 failed\n");
    }

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    if (ret != ESP_OK)
    {
        printf("Could not configure mpu6050\n");
    }
    ret = mpu6050_wake_up(mpu6050);

    if (ret != ESP_OK)
    {
        printf("Could not wake up mpu6050\n");
    }
}

void app_main(void)
{
    printf("Running mpu6050 test tool!\n");

    esp_err_t ret;
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;

    i2c_sensor_mpu6050_init();

    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    if (ret != ESP_OK)
    {
        printf("Get mpu6050 device id failed\n");
        return;
    }

    printf("Initializing complete, device id: %d\n", mpu6050_deviceid);

    while (true)
    {
        if (mpu6050_get_acce(mpu6050, &acce) != ESP_OK)
        {
            printf("Reading mpu6050 acc failed\n");
            break;
        }
        if (mpu6050_get_gyro(mpu6050, &gyro) != ESP_OK)
        {
            printf("Reading mpu6050 gyro failed\n");
            break;
        }

        printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", acce.acce_x, acce.acce_y, acce.acce_z, gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    mpu6050_delete(mpu6050);
    ret = i2c_driver_delete(I2C_MASTER_NUM);
    if (ret != ESP_OK)
    {
        printf("deleting i2c driver failed\n");
    }
}
