#include "debug.h"
#include "drv_time.h"
#include "drv_usb.h"
#include "project.h"

#include "stm32f4xx.h"

extern float pidkp[PIDNUMBER];
extern float pidki[PIDNUMBER];
extern float pidkd[PIDNUMBER];

#ifdef DEBUG
extern debug_type debug;
extern float rx[];

#ifdef RX_FRSKY
#include "rx.h"
extern frsky_bind_data frsky_bind;
#endif
#endif

void systemResetToBootloader(void) {
  *((uint32_t *)0x2001FFFC) = 0xDEADBEEF; // 128KB SRAM STM32F4XX
  NVIC_SystemReset();
}

//This function will be where all usb send/receive coms live
void usb_configurator(uint8_t *data, uint32_t len) {
  if (len) {
    usb_serial_write(data, len);
  }

  for (uint32_t i = 0; i < len; i++) {
    switch (data[i]) {
    case 'R':
      //  The following bits will reboot to DFU upon receiving 'R' (which is sent by BF configurator)
      usb_serial_print("SYSTEM RESET\r\n");
      delay(50 * 1000);
      systemResetToBootloader();
      break;
    case 'P':
      usb_serial_printf("pidkp: %f %f %f\r\n", pidkp[0], pidkp[1], pidkp[2]);
      usb_serial_printf("pidki: %f %f %f\r\n", pidki[0], pidki[1], pidki[2]);
      usb_serial_printf("pidkd: %f %f %f\r\n", pidkd[0], pidkd[1], pidkd[2]);
      break;
#ifdef DEBUG
    case 'D':
      usb_serial_printf("adcfilt: %f\r\n", debug.adcfilt);
      usb_serial_printf("looptime: %f\r\n", debug.timefilt);
      usb_serial_printf("cpu_load: %f\r\n", debug.cpu_load * 100);
      usb_serial_printf("max_cpu_load: %f\r\n", debug.max_cpu_load * 100);
      usb_serial_printf("RX: %f %f %f %f\r\n", rx[0], rx[1], rx[2], rx[3]);
#ifdef RX_FRSKY
      usb_serial_print("FRSKY_BIND\r\n");
      usb_serial_printf("idx: %d\r\n", frsky_bind.idx);
      usb_serial_printf("offset: %d\r\n", frsky_bind.offset);
      usb_serial_printf("tx_id: 0x%x%x\r\n", frsky_bind.tx_id[0], frsky_bind.tx_id[1]);
      usb_serial_printf("hop_data:");
      for (uint32_t i = 0; i < 50; i++) {
        usb_serial_printf(" %x, ", frsky_bind.hop_data[i]);
      }
      usb_serial_print("\r\n");
#endif
      break;
#endif
    }
  }
}