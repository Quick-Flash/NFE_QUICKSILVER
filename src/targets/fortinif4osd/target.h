#include "config.h"

#define F4
#define F405
#define FortiniF4osd

//***************REV 2 ONLY***************

//LEDS
#define LED_NUMBER 2
#define LED1PIN GPIO_Pin_5
#define LED1PORT GPIOB
#define LED1_INVERT
#define LED2PIN GPIO_Pin_6
#define LED2PORT GPIOB
#define BUZZER_PIN GPIO_Pin_4
#define BUZZER_PIN_PORT GPIOB
#define BUZZER_INVERT
#define FPV_PIN GPIO_Pin_13
#define FPV_PORT GPIOA

//GYRO
#define ICM20602_SPI1
#define ICM20602_NSS_PA8
#define ICM20602_INT_PC4
#define USE_DUMMY_I2C
#define GYRO_ID_1 0x12
#define GYRO_ID_2 0xaf
#define GYRO_ID_3 0xac
#define GYRO_ID_4 0x98
#define SENSOR_ROTATE_90_CCW

//RADIO
#define USART_INVERTER_PIN GPIO_Pin_15
#define USART_INVERTER_PORT GPIOC

#define USART_PORTS \
  USART1_PA10PA9    \
  USART3_PB11PB10   \
  USART4_PA1PA0     \
  USART6_PC7PC6

#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT) || defined(RX_UNIFIED_SERIAL)
#define SOFTSPI_NONE
#define RX_USART USART_PORT1
#endif

#ifndef SOFTSPI_NONE
#define RADIO_CHECK
#define SPI_MISO_PIN GPIO_Pin_10
#define SPI_MISO_PORT GPIOA
#define SPI_MOSI_PIN GPIO_Pin_9
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_11
#define SPI_CLK_PORT GPIOB
#define SPI_SS_PIN GPIO_Pin_10
#define SPI_SS_PORT GPIOB
#endif

//OSD
#define ENABLE_OSD
#define MAX7456_SPI3
#define MAX7456_NSS_PB3

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_12
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 10000
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 1000
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PA3
#define MOTOR_PIN1 MOTOR_PIN_PA2
#define MOTOR_PIN2 MOTOR_PIN_PB0
#define MOTOR_PIN3 MOTOR_PIN_PB1