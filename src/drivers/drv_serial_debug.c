// serial for stm32 not used yet
#include "drv_serial.h"

#include <stdio.h>

#include "project.h"

// enable serial driver ( pin SWCLK after calibration)
// WILL DISABLE PROGRAMMING AFTER GYRO CALIBRATION - 2 - 3 seconds after powerup)

// this has to be in config.h
//#define SERIAL_ENABLE

#define SERIAL_BUFFER_SIZE 64

#define SERIAL_BAUDRATE 115200

#ifdef SERIAL_ENABLE

uint8_t buffer[SERIAL_BUFFER_SIZE];
char buffer_start = 0;
char buffer_end = 0;

void USART1_IRQHandler(void) {
  if (buffer_end != buffer_start) {
    LL_USART_TransmitData8(USART1, buffer[buffer_start]);
    buffer_start++;
    buffer_start = buffer_start % (SERIAL_BUFFER_SIZE);
  } else {
    LL_USART_DisableIT_TXE(USART1);
  }
}

void serial_debug_init(void) {

  LL_GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin = LL_GPIO_PIN_14;
  LL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF_1);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  LL_USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.BaudRate = SERIAL_BAUDRATE;
  USART_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStructure.StopBits = LL_USART_STOPBITS_1;
  USART_InitStructure.Parity = LL_USART_PARITY_NONE;
  USART_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStructure.TransferDirection = LL_USART_DIRECTION_TX; //LL_USART_DIRECTION_RX | LL_USART_DIRECTION_TX;

  USART_Init(USART1, &USART_InitStructure);

  //	LL_USART_EnableIT_TXE(USART1);
  USART_Cmd(USART1, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

int fputc(int ch, FILE *f) {
  buffer[buffer_end] = (char)ch;
  buffer_end++;
  buffer_end = buffer_end % (SERIAL_BUFFER_SIZE);

  LL_USART_EnableIT_TXE(USART1);
  return ch;
}

void buffer_add(int val) {
  buffer[buffer_end] = (char)val;
  buffer_end++;
  buffer_end = buffer_end % (SERIAL_BUFFER_SIZE);

  LL_USART_EnableIT_TXE(USART1);
  return;
}

#else
// serial disabled - dummy functions
void serial_debug_init(void) {
}

void buffer_add(int val) {
}

#endif
