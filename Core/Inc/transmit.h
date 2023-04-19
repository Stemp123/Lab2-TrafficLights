#ifndef TRANSMIT_H
#define TRANSMIT_H

#include <string.h>
#include "main.h"

void replace_newline_with_crlf(const char* src, char* dest, size_t dest_size) {
    size_t i = 0, j = 0;
    while (src[i] != '\0' && j < dest_size - 1) {
        if (src[i] == '\n') {
            dest[j++] = '\r';
            if (j < dest_size - 1) {
                dest[j++] = '\n';
            }
        } else {
            dest[j++] = src[i];
        }
        i++;
    }
    dest[j] = '\0';
}

void Transmit(UART_HandleTypeDef *huart, const char* text) {
    char formatted_str[512] = { '\0' };
    replace_newline_with_crlf(text, formatted_str, sizeof(formatted_str));

    uint16_t str_len = strlen(formatted_str);
    HAL_UART_Transmit(huart, (uint8_t*) formatted_str, str_len, HAL_MAX_DELAY);
}

#endif // TRANSMIT_H
