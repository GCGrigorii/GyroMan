#ifndef USART_CMD
#define USART_CMD

#include "stdint.h"

typedef struct{
  uint8_t state;
  uint8_t fcn;
} usart_cmd_;

char usart_cmd (volatile char*, uint16_t, usart_cmd_*);

#endif
