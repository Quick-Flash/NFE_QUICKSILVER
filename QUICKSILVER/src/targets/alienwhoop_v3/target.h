#include "config.h"

#define F405
#define Alienwhoop_V3

//LEDS
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_12
#define LED1PORT GPIOC
#define LED2PIN GPIO_Pin_2
#define LED2PORT GPIOD
//#define LED1_INVERT
#define LED2_INVERT
#define BUZZER_PIN GPIO_Pin_2
#define BUZZER_PIN_PORT GPIOA
#define BUZZER_INVERT
#define FPV_PIN GPIO_Pin_13
#define FPV_PORT GPIOA

//SPI, I2C & GYRO
#define MPU6XXX_SPI1
#define MPU6XXX_NSS_PA4
#define MPU6XXX_INT_PC14
#define USE_DUMMY_I2C //todo: soft i2c is working for f4 but I dont think i have done hardware i2c - disabled for now since all f4 boards use spi gyro
//#define I2C_SDAPIN GPIO_Pin_10
//#define I2C_SDAPORT GPIOA
//#define I2C_SCLPIN GPIO_Pin_9
//#define I2C_SCLPORT GPIOA
//#define I2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69
#define GYRO_ID_1 0x70
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x71
//#define GYRO_ID_4 0x72
#define SENSOR_ROTATE_90_CW

// SPI PINS DEFINITONS & RADIO
#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT)
#define USART1_PA10PA9
#define USART2_PA3PA2
#define USART3_PC11PC10
#define SOFTSPI_NONE
#endif
#ifndef SOFTSPI_NONE
#define RADIO_CHECK
#define SPI_MISO_PIN GPIO_Pin_10
#define SPI_MISO_PORT GPIOA
#define SPI_MOSI_PIN GPIO_Pin_9
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_6
#define SPI_CLK_PORT GPIOC
#define SPI_SS_PIN GPIO_Pin_7
#define SPI_SS_PORT GPIOC
#endif

// OSD
#define ENABLE_OSD
#define MAX7456_SPI2
#define MAX7456_NSS_PB12

//VOLTAGE DIVIDER
//#define DISABLE_ADC
#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_12
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 2000
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 1000
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
#define MOTOR0_PIN_PC6
#define MOTOR1_PIN_PC7
#define MOTOR2_PIN_PC8
#define MOTOR3_PIN_PC9

// PWM PIN INITIALIZATION
//#define PWM_PA0
//#define PWM_PA1
//#define PWM_PA2
//#define PWM_PA3
//#define PWM_PA4
//#define PWM_PA5
//#define PWM_PA6
//#define PWM_PA7
//#define PWM_PA8
//#define PWM_PA9
//#define PWM_PA10
//#define PWM_PA11
//#define PWM_PB0
//#define PWM_PB1
#define PWM_PC6
#define PWM_PC7
#define PWM_PC8
#define PWM_PC9
