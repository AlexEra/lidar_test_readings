/*
 * usart_ring.h
 *
 *  Created on: 19 авг. 2019 г.
 *      Author: dima
 *      Modified by Alera at 1st March 2024
 */

#ifndef USART_RING_H_
#define USART_RING_H_

#include "usart.h"


typedef struct {
	UART_HandleTypeDef *usart_rx;
	uint16_t size;
	uint16_t head;
	uint16_t tail;
	uint8_t *buffer;
} usart_ring_t;

uint8_t usart_ring_init(usart_ring_t *ur, UART_HandleTypeDef *usart_rx, size_t size);

void usart_ring_destroy(usart_ring_t *ur);

/**
 * @brief Clearing the ring buffer
 * */
void usart_ring_clear(usart_ring_t *ur);

/**
 * @brief Callback interrupt function
 * It have to be used in USARTx_IRQHandler placed into stm32fxxx_it.c file
 * WARNING! return instruction should be written after calling of this function
 * for faster processing. Do not forget to include header file!
 * */
void irq_callback(usart_ring_t *ur);

/**
 * @brief Checking the ring buffer for the new data
 * @return Amount of bytes saved into the ring buffer
 * */
uint16_t usart_ring_available(usart_ring_t *ur);

/**
 * @brief Reading the tail byte (not last, first incoming)
 * @return Tail byte value
 * */
uint8_t usart_ring_read(usart_ring_t *ur);

#endif /* USART_RING_H_ */
