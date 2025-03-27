#include <stdio.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

// I2C Pin number與 MPU9250 地址
#define I2C_SCL_PIN 21
#define I2C_SDA_PIN 26
#define MPU9250_SENSOR_ADDR 0x68
#define I2C_MASTER_WRITE 0x00
#define I2C_MASTER_READ 0x01

// Grafcet 狀態變數
int X0 = 1, X1 = 0, X1_1 = 0, X2 = 0, X0_3 = 1, X1_3 = 0, X2_3 = 0, X3_3 = 0;
int R0 = 0, R1 = 0, R1_1 = 0, R2 = 0, R0_3 = 0, R1_3 = 0, R2_3 = 0, R3_3 = 0;
int i = 0; // 暫存器讀取次數
uint8_t accel_data[6] = {0}; // 用於保存XYZ軸上的暫存器資料。
uint8_t temp_data = 0;
uint8_t* cur_accel_addr = NULL;
int break_flag = 0;


void grafcet0();
void datapath0();
void action0();
void i2c_start_action();
void read_mpu_start_action();
void read_mpu_register_action();
void process_accel_data_action();

void datapath1();
void grafcet1();
void send_start_signal_action();
void write_sensor_address_action(uint8_t rw_flag);
void write_register_address_action(uint8_t reg);
void read_sensor_address_action(uint8_t rw_flag);
void read_register_data_action(uint8_t *data);

void i2c_init()
{
    gpio_set_direction(I2C_SCL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(I2C_SDA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(I2C_SCL_PIN, 1);
    gpio_set_level(I2C_SDA_PIN, 1);
}

void i2c_start()
{
    gpio_set_level(I2C_SDA_PIN, 1);
    gpio_set_level(I2C_SCL_PIN, 1);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_SDA_PIN, 0);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_SCL_PIN, 0);
}

void i2c_stop()
{
    gpio_set_level(I2C_SDA_PIN, 0);
    gpio_set_level(I2C_SCL_PIN, 1);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_SDA_PIN, 1);
    esp_rom_delay_us(5);
}

int i2c_write_byte(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        gpio_set_level(I2C_SDA_PIN, (data >> (7 - i)) & 0x01);//MSB first
        esp_rom_delay_us(5);
        gpio_set_level(I2C_SCL_PIN, 1);// 拉高SCL，讓slave讀資料
        esp_rom_delay_us(5);
        gpio_set_level(I2C_SCL_PIN, 0);
    }

    gpio_set_direction(I2C_SDA_PIN, GPIO_MODE_INPUT);
    gpio_set_level(I2C_SCL_PIN, 1);
    esp_rom_delay_us(5);
    int ack = gpio_get_level(I2C_SDA_PIN);
    gpio_set_level(I2C_SCL_PIN, 0);
    gpio_set_direction(I2C_SDA_PIN, GPIO_MODE_OUTPUT);

    return ack == 0 ? 0 : -1;
}

// Grafcet Actions
void action0()
{
    printf("Action 0: Initialize I2C GPIO\n");
    i2c_init_action();
    R0 = 1;
}


void read_mpu_start_action()
{
    i2c_start();
    printf("Action1: Start Reading MPU9250\n");
    i = 0;
    R1 = 1;
}

int i2c_read_byte_temp(uint8_t reg, int i){
    temp_data = reg;
    break_flag = 0;
    printf("\n");
    while(!break_flag){
    datapath1();//sub-grafcet
    grafcet1();//sub-grafcet
    printf("X20 = %d, X21 = %d, X22 = %d, X23 = %d\n", X0_3, X1_3, X2_3, X3_3);
    }
    return 0;
}
void datapath1() {
    if (X0_3 == 1) {
        // send_start_signal_action();
        write_sensor_address_action(I2C_MASTER_WRITE);
        R0_3 = 1;
    }
    if (X1_3 == 1) {
        write_register_address_action(temp_data);
        R0_3 = 0;
        R1_3 = 1;
    }
    if (X2_3 == 1) {
        // send_start_signal_action();
        read_sensor_address_action(I2C_MASTER_READ);
        R0_3 = 0;
        R2_3 = 1;
    }
    if (X3_3 == 1) {
        read_register_data_action(&accel_data[i]);
        R2_3 = 0;
        R3_3 = 1;
    }
}

void grafcet1() {
    if (X0_3 == 1 && R0_3) {
        X0_3 = 0;
        X1_3 = 1;
        return;
    }
    if (X1_3 == 1 && R1_3) {
        X1_3 = 0;
        X2_3 = 1;
        return;
    }
    if (X2_3 == 1 && R2_3) {
        X2_3 = 0;
        X3_3 = 1;
        return;
    }
    if (X3_3 == 1 && R3_3) {
        X3_3 = 0;
        X0_3 = 1;
        break_flag = 1;
        return;
    }
}

void send_start_signal_action()
{
    printf("Action: Send Start Signal\n");
    i2c_start();
}


void write_sensor_address_action(uint8_t rw_flag)
{
    send_start_signal_action();
    printf("Action20: Write Sensor Address 0x%X\n", MPU9250_SENSOR_ADDR);
    i2c_write_byte(MPU9250_SENSOR_ADDR << 1 | rw_flag);
}

void write_register_address_action(uint8_t reg) {
    printf("Action21: Write Register Address 0x%X\n", reg);
    if (i2c_write_byte(reg) != 0) {
        printf("Failed to write register address\n");
        i2c_stop();
    }
}

void read_sensor_address_action(uint8_t rw_flag) {
    send_start_signal_action();
    printf("Action22: Read Sensor Address 0x%X\n", MPU9250_SENSOR_ADDR);
    // if (i2c_write_byte(MPU9250_SENSOR_ADDR << 1 | I2C_MASTER_READ) != 0) {
    //     printf("Failed to read sensor address\n");
    //     i2c_stop();
    // }
    i2c_write_byte(MPU9250_SENSOR_ADDR << 1 | rw_flag);
}

void read_register_data_action(uint8_t *data) {
    printf("Action23: Read Register Data\n");
    gpio_set_direction(I2C_SDA_PIN, GPIO_MODE_INPUT);
    *data = 0;
    for (int i = 0; i < 8; i++) {
        gpio_set_level(I2C_SCL_PIN, 1);
        esp_rom_delay_us(5);
        *data = (*data << 1) | gpio_get_level(I2C_SDA_PIN);
        gpio_set_level(I2C_SCL_PIN, 0);
        esp_rom_delay_us(5);
    }
    gpio_set_direction(I2C_SDA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(I2C_SDA_PIN, 1);
    gpio_set_level(I2C_SCL_PIN, 1);
    esp_rom_delay_us(5);
    i2c_stop();
    return;
}


void read_mpu_register_action()
{
    if (i < 6) {
        printf("Action2: Read Register 0x%X\n", 0x3B + i);
        if (i2c_read_byte_temp(0x3B + i, i) == 0) {
            i++;
            if (i == 6) {
                R1_1 = 1;
            }
        } else {
            printf("Failed to read register 0x%X\n", 0x3B + i);
        }
    }
}


void process_accel_data_action()
{
    printf("Action: Process Accelerometer Data\n");
    int16_t accel_x = (accel_data[0] << 8) | accel_data[1];
    int16_t accel_y = (accel_data[2] << 8) | accel_data[3];
    int16_t accel_z = (accel_data[4] << 8) | accel_data[5];
    printf("Accel X: %d, Y: %d, Z: %d\n", accel_x, accel_y, accel_z);
    R2 = 1;
}

void grafcet0()
{
    if (X0 == 1 && R0) {
        X0 = 0;
        X1 = 1;
        return;
    }

    if (X1 == 1 && R1) {
        X1 = 0;
        X1_1 = 1;
        return;
    }

    if (X1_1 == 1 && R1_1) {
        if (i < 6) {
            return; // 繼續讀取暫存器
        } else if (R1_1) {
            X1_1 = 0;
            X2 = 1;
            return;
        }
    }

    if (X2 == 1 && R2) {
        X2 = 0;
        X0 = 1;
        return;
    }
}

void datapath0()
{
    if (X0 == 1) {
        action0();
    }
    if (X1 == 1) read_mpu_start_action();
    if (X1_1 == 1) {
        if (i < 6) {
            read_mpu_register_action();
        }
    }
    if (X2 == 1) process_accel_data_action();
}

// Main Function
void app_main()
{
    printf("\n");
    while (1) {
        datapath0();
        grafcet0();
        // printf("\nX0 = %d, X1 = %d, X2 = %d, X3 = %d\n", X0, X1, X1_1, X2);
        vTaskDelay(pdMS_TO_TICKS(1000));//10ms = 0.01s
    }
}
