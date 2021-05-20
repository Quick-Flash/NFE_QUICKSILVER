#include "config.h"
#include "config_helper.h"

#define F4
#define F405
#define CLRacing_F4

//PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PC10PC11PC12

#define USART_PORTS \
  USART1_PA10PA9    \
  USART3_PB11PB10   \
  USART4_PA1PA0     \
  USART6_PC7PC6

// no bueno on CLRacingF4s
//#define USB_DETECT_PIN GPIO_Pin_5
//#define USB_DETECT_PORT GPIOC

//LEDS
#define LED_NUMBER 1
#define LED1PIN PIN_B5
#define LED1_INVERT
#define BUZZER_PIN PIN_B4
#define BUZZER_INVERT
#define FPV_PIN PIN_A13

//GYRO
#define MPU6XXX_SPI_PORT SPI_PORT1
#define MPU6XXX_NSS PIN_A4
#define MPU6XXX_INT PIN_C4
#define GYRO_ID_1 0x68
#define SENSOR_ROTATE_90_CW

//RADIO
#define USART1_INVERTER_PIN PIN_C0

#ifdef SERIAL_RX
#define RX_USART USART_PORT1
#define SOFTSPI_NONE
#endif
#ifndef SOFTSPI_NONE
#define RADIO_CHECK
#define SPI_MISO_PIN GPIO_Pin_10
#define SPI_MISO_PORT GPIOA
#define SPI_MOSI_PIN GPIO_Pin_9
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin11
#define SPI_CLK_PORT GPIOB
#define SPI_SS_PIN GPIO_Pin_10
#define SPI_SS_PORT GPIOB
#endif

//OSD
#define ENABLE_OSD
#define MAX7456_SPI_PORT SPI_PORT3
#define MAX7456_NSS PIN_A15

// #define USE_M25P16
// #define M25P16_SPI_PORT SPI_PORT3
// #define M25P16_NSS_PIN PIN_B3

//VOLTAGE DIVIDER
#define BATTERYPIN PIN_C2
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
