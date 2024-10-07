#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "Fusion.h"

#define MPU_ADDRESS 0x68
#define I2C_PORT i2c0
#define UART_PORT uart0
#define SAMPLE_PERIOD (0.01f) // Defina com o valor correto para a sua aplicação
#define THRESHOLD_CLICK 5.0f  // Defina o valor do threshold para detectar o "mouse click"
#define DEADZONE 2.0f         // Deadzone para movimentos
#define SENSITIVITY_X 5.0f    // Sensibilidade para o movimento horizontal (yaw)
#define SENSITIVITY_Y 5.0f    // Sensibilidade para o movimento vertical (roll)

void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, buf, 2, false);
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x43;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x41;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 2, false);
    *temp = buffer[0] << 8 | buffer[1];
}

int main() {
    stdio_init_all();
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    uart_init(UART_PORT, 115200);
    gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    float previous_yaw = 0.0f;
    float previous_roll = 0.0f;

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        // Converte dados brutos
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f
        };

        // Atualiza a fusão de dados
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        // Calcula a orientação (Roll, Pitch, Yaw)
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Aplica a deadzone e a sensibilidade
        float delta_yaw = euler.angle.yaw - previous_yaw;
        float delta_roll = euler.angle.roll - previous_roll;

        if (fabs(delta_yaw) > DEADZONE) {
            int move_x = (int)(delta_yaw * SENSITIVITY_X);
            char buffer[20];
            snprintf(buffer, sizeof(buffer), "X:%d\n", move_x);
            uart_write_blocking(UART_PORT, (const uint8_t *)buffer, strlen(buffer));
            previous_yaw = euler.angle.yaw;
        }

        if (fabs(delta_roll) > DEADZONE) {
            int move_y = (int)(delta_roll * SENSITIVITY_Y);
            char buffer[20];
            snprintf(buffer, sizeof(buffer), "Y:%d\n", move_y);
            uart_write_blocking(UART_PORT, (const uint8_t *)buffer, strlen(buffer));
            previous_roll = euler.angle.roll;
        }

        // Detecta movimento repentino no eixo X (Roll) para acionar o clique do mouse
        if (accelerometer.axis.y > 1.5) {
            // Envia um sinal de clique pela UART
            uart_puts(UART_PORT, "C:1\n");  // Sinal de clique
        }

        sleep_ms(10);
    }

    return 0;
}