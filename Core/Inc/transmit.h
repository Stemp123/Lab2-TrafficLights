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

/*
void test_circular_queue() {
    Transmit(&huart1, "Initialized queue:\n");
    evq_init();
    Transmit(&huart1, "<print_evq>");

    Transmit(&huart1, "Pushing events 1 to 5:\n");
    for (int i = 1; i <= 5; i++) {
        evq_push_back(i);
    }
    Transmit(&huart1, "<print_evq>");

    Transmit(&huart1, "Popping 3 events:\n");
    char buffer[32];
    for (int i = 0; i < 3; i++) {
        sprintf(buffer, "Popped: %d\n", evq_pop_front());
        Transmit(&huart1, buffer);
    }
    Transmit(&huart1, "<print_evq>");

    Transmit(&huart1, "Pushing events 6 to 10:\n");
    for (int i = 6; i <= 10; i++) {
        evq_push_back(i);
    }
    Transmit(&huart1, "<print_evq>");

    Transmit(&huart1, "Popping 5 events:\n");
    for (int i = 0; i < 5; i++) {
        sprintf(buffer, "Popped: %d\n", evq_pop_front());
        Transmit(&huart1, buffer);
    }
    Transmit(&huart1, "<print_evq>");

    Transmit(&huart1, "Pushing events 11 to 12:\n");
    for (int i = 11; i <= 12; i++) {
        evq_push_back(i);
    }
    Transmit(&huart1, "<print_evq>");
}*/
