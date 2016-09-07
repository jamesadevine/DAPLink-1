#include <stdint.h>

#ifndef TARGET_FLASH_ERASE_CHIP_OVERRIDE
#define  MICROBIT_PAGE_BUFFER_SIZE  1024
#else
#define MICROBIT_PAGE_BUFFER_SIZE   2048
#endif

extern uint8_t microbit_page_buffer[MICROBIT_PAGE_BUFFER_SIZE];

/**
  * Performs an in buffer reverse of a given char array.
  *
  * @param s the string to reverse.
  *
  * @return MICROBIT_OK, or MICROBIT_INVALID_PARAMETER.
  */
int string_reverse(char *s);

/**
  * Converts a given integer into a string representation.
  *
  * @param n The number to convert.
  *
  * @param s A pointer to the buffer where the resulting string will be stored.
  *
  * @return MICROBIT_OK, or MICROBIT_INVALID_PARAMETER.
  */
int itoa(int n, char *s);


int strnlen(const char *s, uint16_t max_len);
