/*
 * usart_ring.c
 *
 *  Created on: 19 авг. 2019 г.
 *      Author: dima
 *      Modified by Alera at 1st March 2024
 */

#include <stdlib.h>
#include <usart_ring.h>

uint8_t usart_ring_init(usart_ring_t *ur, UART_HandleTypeDef *usart_rx, size_t size) {
	ur->buffer = (uint8_t *) malloc(size);
	if (ur->buffer == NULL) {
		return 0;
	}
	ur->head = 0;
	ur->tail = 0;
	ur->size = size;
	ur->usart_rx = usart_rx;
	return 1;
}

void usart_ring_destroy(usart_ring_t *ur) {
	__HAL_UART_DISABLE_IT(ur->usart_rx, UART_IT_RXNE);
	free(ur->buffer);
	ur->buffer = NULL;
	ur->head = 0;
	ur->tail = 0;
	__HAL_UART_ENABLE_IT(ur->usart_rx, UART_IT_RXNE);
}

void usart_ring_clear(usart_ring_t *ur) {
	__HAL_UART_DISABLE_IT(ur->usart_rx, UART_IT_RXNE);
	ur->head = 0;
	ur->tail = 0;
	__HAL_UART_ENABLE_IT(ur->usart_rx, UART_IT_RXNE);
}

void irq_callback(usart_ring_t *ur) {
	if ((ur->usart_rx->Instance->SR & USART_SR_RXNE) != RESET) {
		uint8_t rbyte = (uint8_t) (ur->usart_rx->Instance->DR & (uint8_t) 0x00FF); // reading byte from the register
		uint16_t i = (uint16_t) (ur->head + 1) % ur->size;

		if (i != ur->tail) {
			ur->buffer[ur->head] = rbyte;
			ur->head = i;
		}
	}
}

uint16_t usart_ring_available(usart_ring_t *ur) {
	return (uint16_t)(((uint32_t)ur->size + ur->head - ur->tail) % ur->size);
}

uint8_t usart_ring_read(usart_ring_t *ur) {
	if (ur->head == ur->tail) {
		return 0;
	} else {
		uint8_t c = ur->buffer[ur->tail];
		ur->tail = (uint16_t) (ur->tail + 1) % ur->size;
		return c;
	}
}
