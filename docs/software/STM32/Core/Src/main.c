/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — ICM-45686 reaction-wheel bicycle balancer
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "math.h"
#include "usb_device.h"
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"

#define CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

#define ICM45686_REG_DATA_BASE        0x00
#define ICM45686_REG_PWR_MGMT0        0x10
#define ICM45686_REG_INT1_CONFIG0     0x16
#define ICM45686_REG_INT1_STATUS0     0x19
#define ICM45686_REG_ACCEL_CONFIG0    0x1B
#define ICM45686_REG_GYRO_CONFIG0     0x1C
#define ICM45686_REG_WHO_AM_I         0x72
#define ICM45686_REG_IREG_ADDR_15_8   0x7C
#define ICM45686_REG_IREG_ADDR_7_0    0x7D
#define ICM45686_REG_IREG_DATA        0x7E
#define ICM45686_SREG_CTRL_ADDR_H     0xA2
#define ICM45686_SREG_CTRL_ADDR_L     0x67
#define ICM45686_SREG_DATA_BIG_ENDIAN (1u << 1)
#define ICM45686_WHOAMI_VALUE         0xE9
#define ICM45686_ACCEL_2G_200HZ       0x48
#define ICM45686_GYRO_125DPS_200HZ    0x58
#define ICM45686_PWR_ACCEL_GYRO_LN    0x0F
#define ICM45686_INT1_CFG_EN_DRDY     (1u << 2)
#define ICM45686_INT1_STATUS_DRDY     (1u << 2)

/* USER CODE END Includes */

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN 0 */

void ODESC_Send(char *cmd)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 5);
}

void USB_Print(char *msg)
{
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

void IMU_Write(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = { reg & 0x7F, data };
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    CS_HIGH();
}

uint8_t IMU_Read(uint8_t reg)
{
    uint8_t tx[2] = { reg | 0x80, 0x00 };
    uint8_t rx[2] = {0};
    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    CS_HIGH();
    return rx[1];
}

void IMU_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[len + 1];
    uint8_t rx[len + 1];
    tx[0] = reg | 0x80;
    memset(tx + 1, 0x00, len);
    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len + 1, HAL_MAX_DELAY);
    CS_HIGH();
    memcpy(buf, rx + 1, len);
}

void IMU_WriteIREG(uint8_t addr_h, uint8_t addr_l, uint8_t data)
{
    uint8_t tx[4];
    tx[0] = ICM45686_REG_IREG_ADDR_15_8 & 0x7F;
    tx[1] = addr_h;
    tx[2] = addr_l;
    tx[3] = data;
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 4, HAL_MAX_DELAY);
    CS_HIGH();
    HAL_Delay(1);
}

void IMU_Init(void)
{
    HAL_Delay(10);

    uint8_t who = IMU_Read(ICM45686_REG_WHO_AM_I);
    if (who != ICM45686_WHOAMI_VALUE) return;

    IMU_WriteIREG(ICM45686_SREG_CTRL_ADDR_H,
                  ICM45686_SREG_CTRL_ADDR_L,
                  ICM45686_SREG_DATA_BIG_ENDIAN);

    uint8_t int1_cfg0 = IMU_Read(ICM45686_REG_INT1_CONFIG0);
    int1_cfg0 |= ICM45686_INT1_CFG_EN_DRDY;
    IMU_Write(ICM45686_REG_INT1_CONFIG0, int1_cfg0);

    IMU_Write(ICM45686_REG_ACCEL_CONFIG0, ICM45686_ACCEL_2G_200HZ);
    IMU_Write(ICM45686_REG_GYRO_CONFIG0,  ICM45686_GYRO_125DPS_200HZ);
    IMU_Write(ICM45686_REG_PWR_MGMT0,     ICM45686_PWR_ACCEL_GYRO_LN);

    HAL_Delay(50);
}

#define GYRO_LSB_PER_DPS  262.0f
#define ACCEL_SENS        16384.0f
#define DEG2RAD           0.01745329251f
#define DT                0.005f

#define CF_ALPHA          0.9995f
#define GYRO_LPF_ALPHA    0.1f
#define FALL_LIMIT        0.1f

static float angle      = 0.0f;
static float gy_bias    = 0.0f;
static float gy_filtered = 0.0f;
static float setpoint   = 0.0f;

float Kp = 650.0f;
float Ki = 0.0f;
float Kd = 65.0f;

static float integral = 0.0f;
#define INTEGRAL_LIMIT  1.0f
#define MAX_TORQUE      10.0f

#define OUTER_INTERVAL_TICKS  10
#define SETPOINT_LIMIT        1.0f
#define TORQUE_DEADBAND       0.0f
#define SETPOINT_SLEW_RATE    0.0001f

float K_outer = 0.01f;

static float torque_sum = 0.0f;

#define VBUS_INTERVAL_TICKS   2000
#define VBUS_LOW_THRESHOLD    20.0f
#define BLINK_PERIOD_TICKS    25

static uint8_t  vbus_low   = 0;
static uint32_t blink_tick = 0;

float clamp(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

void IMU_CalibrateGyro(void)
{
    int32_t sum = 0;
    const int samples = 500;
    for (int i = 0; i < samples; i++)
    {
        uint8_t buf[14];
        IMU_ReadMulti(ICM45686_REG_DATA_BASE, buf, 14);
        int16_t gy = (int16_t)((buf[8] << 8) | buf[9]);
        sum += gy;
        HAL_Delay(2);
    }
    gy_bias = (float)sum / samples;
}

void IMU_InitAngle(void)
{
    const int samples = 500;
    float angle_sum = 0.0f;
    for (int i = 0; i < samples; i++)
    {
        uint8_t buf[14];
        IMU_ReadMulti(ICM45686_REG_DATA_BASE, buf, 14);
        int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);
        float ax_g = ax_raw / ACCEL_SENS;
        float az_g = az_raw / ACCEL_SENS;
        angle_sum += atan2f(ax_g, az_g);
        HAL_Delay(2);
    }
    angle = angle_sum / samples;
}

void ODESC_InitTorqueMode(void)
{
    ODESC_Send("w axis0.controller.config.vel_limit 30\n");   HAL_Delay(10);
    ODESC_Send("w axis0.controller.config.control_mode 1\n"); HAL_Delay(10);
    ODESC_Send("w axis0.controller.config.input_mode 1\n");   HAL_Delay(10);
    ODESC_Send("w axis0.requested_state 8\n");                HAL_Delay(100);
}

float ODESC_GetVBus(void)
{
    uint8_t flush;
    while (HAL_UART_Receive(&huart1, &flush, 1, 1) == HAL_OK);

    ODESC_Send("r vbus_voltage\n");

    char resp[32] = {0};
    uint8_t c;
    uint8_t idx = 0;
    uint32_t deadline = HAL_GetTick() + 10;
    while (HAL_GetTick() < deadline && idx < sizeof(resp) - 1)
    {
        if (HAL_UART_Receive(&huart1, &c, 1, 1) == HAL_OK)
        {
            if (c == '\n') break;
            resp[idx++] = c;
        }
    }
    char *end;
    return strtof(resp, &end);
}
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USB_DEVICE_Init();
    MX_SPI1_Init();

    /* USER CODE BEGIN 2 */

    IMU_Init();
    IMU_CalibrateGyro();
    IMU_InitAngle();
    ODESC_InitTorqueMode();

    /* USER CODE END 2 */

    static uint32_t outer_tick = 0;
    static uint32_t vbus_tick  = 0;
    // static uint32_t print_tick = 0;

    while (1)
    {
        /* USER CODE BEGIN WHILE */

        uint8_t status = IMU_Read(ICM45686_REG_INT1_STATUS0);
        if (!(status & ICM45686_INT1_STATUS_DRDY))
            continue;

        uint8_t buf[14];
        IMU_ReadMulti(ICM45686_REG_DATA_BASE, buf, 14);

        int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);
        float ax_g = ax_raw / ACCEL_SENS;
        float az_g = az_raw / ACCEL_SENS;

        int16_t gy_raw = (int16_t)((buf[8] << 8) | buf[9]);
        float gy_dps  = ((float)gy_raw - gy_bias) / GYRO_LSB_PER_DPS;
        float gy_rad  = gy_dps * DEG2RAD;

        gy_filtered = GYRO_LPF_ALPHA * gy_rad + (1.0f - GYRO_LPF_ALPHA) * gy_filtered;

        float accel_angle = atan2f(ax_g, az_g);
        angle = CF_ALPHA * (angle + gy_rad * DT) - (1.0f - CF_ALPHA) * accel_angle;

        if (fabsf(angle) > FALL_LIMIT)
        {
            ODESC_Send("c 0 0.000\n");
            integral    = 0.0f;
            gy_filtered = 0.0f;
            torque_sum  = 0.0f;
            outer_tick  = 0;
            continue;
        }

        float error = setpoint - angle;
        integral = clamp(integral + error * DT, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        float output = (Kp * error) + (Ki * integral) - (Kd * gy_filtered);
        output = clamp(output, -MAX_TORQUE, MAX_TORQUE);

        int torque_milli = (int)(output * 1000.0f);
        int torque_abs   = abs(torque_milli);
        char cmd[32];
        if (torque_milli < 0)
            snprintf(cmd, sizeof(cmd), "c 0 -%d.%03d\n",
                     torque_abs / 1000, torque_abs % 1000);
        else
            snprintf(cmd, sizeof(cmd), "c 0 %d.%03d\n",
                     torque_abs / 1000, torque_abs % 1000);
        ODESC_Send(cmd);

        torque_sum += output;
        outer_tick++;
        if (outer_tick >= OUTER_INTERVAL_TICKS)
        {
            float avg_torque = torque_sum / outer_tick;
            torque_sum = 0.0f;
            outer_tick = 0;

            if (fabsf(avg_torque) > TORQUE_DEADBAND)
            {
                float correction = K_outer * avg_torque;
                correction = clamp(correction, -SETPOINT_SLEW_RATE, SETPOINT_SLEW_RATE);
                setpoint  += correction;
                setpoint   = clamp(setpoint, -SETPOINT_LIMIT, SETPOINT_LIMIT);
            }
        }

        /*print_tick++;
        if (print_tick >= 200)
        {
            print_tick = 0;

            int angle_micro = (int)(angle    * 10000.0f);
            int sp_micro    = (int)(setpoint * 10000.0f);

            char msg[128];
            snprintf(msg, sizeof(msg),
                "angle: %c%d.%04d rad | setpoint: %c%d.%04d rad\n",
                (angle_micro < 0 ? '-' : '+'), abs(angle_micro) / 10000, abs(angle_micro) % 10000,
                (sp_micro    < 0 ? '-' : '+'), abs(sp_micro)    / 10000, abs(sp_micro)    % 10000);
            USB_Print(msg);
        }*/

        vbus_tick++;
        if (vbus_tick >= VBUS_INTERVAL_TICKS)
        {
            vbus_tick = 0;
            float vbus = ODESC_GetVBus();
            vbus_low = (vbus > 0.0f && vbus < VBUS_LOW_THRESHOLD) ? 1 : 0;
        }

        blink_tick++;
        if (!vbus_low)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            blink_tick = 0;
        }
        else if (blink_tick >= BLINK_PERIOD_TICKS)
        {
            blink_tick = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
        /* USER CODE END WHILE */
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 15;
    RCC_OscInitStruct.PLL.PLLN            = 144;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 5;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef spi_gpio = {0};
    spi_gpio.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    spi_gpio.Mode      = GPIO_MODE_AF_PP;
    spi_gpio.Pull      = GPIO_NOPULL;
    spi_gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    spi_gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &spi_gpio);

    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
