#include "../inc/usart_cmd.h"

#include <string.h>
#include <stdio.h>
//usart_cmd_* UCH;

char usart_cmd (volatile char* str, uint16_t str_size, usart_cmd_* UCH) {
  str = strupr(str);
  if (!strncmp(str, "LED\r", 4)) { UCH->fcn = 1; }
  else if (!strncmp(str, "KP\r", 3)) { UCH->fcn = 2; UCH->state = 0x0f; }
  else if (!strncmp(str, "KI\r", 3)) { UCH->fcn = 3; UCH->state = 0x0f; }
  else if (!strncmp(str, "KD\r", 3)) { UCH->fcn = 4; UCH->state = 0x0f; }
  else if (!strncmp(str, "UP\r", 3)) { UCH->fcn = 5; }
  else if (!strncmp(str, "KL\r", 4)) {UCH->fcn = 6; }
  else if (!strncmp(str, "KLC\r", 4)) {UCH->fcn = 7; }
  else if (!strncmp(str, "CMD\r", 4)) { UCH->state = 0xff; }
  else if (!strncmp(str, "CMDEX\r", 6)) { UCH->state = 0x00; }
  return "\0";
}

float get_num(char* str, char chr, uint8_t len) {
  char num[20];
  for (uint8_t i = 0; i < len & i < 20; i++) {
    if (str[i] == chr) {
      strncpy(&num, &str, i+1);
      return atof(num);
    }
  }
  return 1;
}
