/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file spiuart.c
 *
 * Serial communication for the RPI to UART.
 *
 */

#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

/* XXX might be able to prune these */
#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>
#include <stm32.h>

//#define DEBUG
#include "px4io.h"

#define SPIUART_TX_BUF_SIZE 64
#define SPIUART_RX_BUF_SIZE 256

static DMA_HANDLE	spiuart_tx_dma;
static DMA_HANDLE	spiuart_rx_dma;

static int		serial_interrupt(int irq, void *context);
static void		spiuart_dma_reset(void);

static uint8_t spiuart_tx_buf[SPIUART_TX_BUF_SIZE];
static uint8_t spiuart_rx_buf[SPIUART_RX_BUF_SIZE];

static uint32_t rxdmanext = 0;

#define SPIUART_SERIAL_BASE     STM32_USART2_BASE
#define SPIUART_SERIAL_VECTOR	STM32_IRQ_USART2
#define SPIUART_SERIAL_TX_GPIO	GPIO_USART2_TX
#define SPIUART_SERIAL_RX_GPIO	GPIO_USART2_RX
#define SPIUART_SERIAL_TX_DMA	DMACHAN_USART2_TX
#define SPIUART_SERIAL_RX_DMA	DMACHAN_USART2_RX
#define SPIUART_SERIAL_CLOCK	STM32_PCLK1_FREQUENCY
#define SPIUART_SERIAL_BITRATE	57600

/* serial register accessors */
#define SPIUART_REG(_x)		(*(volatile uint32_t *)(SPIUART_SERIAL_BASE + _x))
#define SPIUART_rSR         SPIUART_REG(STM32_USART_SR_OFFSET)
#define SPIUART_rDR         SPIUART_REG(STM32_USART_DR_OFFSET)
#define SPIUART_rBRR		SPIUART_REG(STM32_USART_BRR_OFFSET)
#define SPIUART_rCR1		SPIUART_REG(STM32_USART_CR1_OFFSET)
#define SPIUART_rCR2		SPIUART_REG(STM32_USART_CR2_OFFSET)
#define SPIUART_rCR3		SPIUART_REG(STM32_USART_CR3_OFFSET)
#define SPIUART_rGTPR		SPIUART_REG(STM32_USART_GTPR_OFFSET)

void
spiuart_init(uint32_t baudrate)
{
    /* allocate DMA */
    spiuart_tx_dma = stm32_dmachannel(SPIUART_SERIAL_TX_DMA);
    spiuart_rx_dma = stm32_dmachannel(SPIUART_SERIAL_RX_DMA);
    
    /* configure pins for serial use */
    stm32_configgpio(SPIUART_SERIAL_TX_GPIO);
    stm32_configgpio(SPIUART_SERIAL_RX_GPIO);
    
    /* reset and configure the UART */
    SPIUART_rCR1 = 0;
    SPIUART_rCR2 = 0;
    SPIUART_rCR3 = 0;
    
    /* clear status/errors */
    (void)SPIUART_rSR;
    (void)SPIUART_rDR;
    
    /* configure line speed */
    uint32_t usartdiv32 = 0;
    if (baudrate != 0) {
        usartdiv32 = SPIUART_SERIAL_CLOCK / (baudrate / 2);
    } else {
        usartdiv32 = SPIUART_SERIAL_CLOCK / (SPIUART_SERIAL_BITRATE / 2);
    }
    uint32_t mantissa = usartdiv32 >> 5;
    uint32_t fraction = (usartdiv32 - (mantissa << 5) + 1) >> 1;
    SPIUART_rBRR = (mantissa << USART_BRR_MANT_SHIFT) | (fraction << USART_BRR_FRAC_SHIFT);
    
    /* connect our interrupt */
    irq_attach(SPIUART_SERIAL_VECTOR, serial_interrupt);
    up_enable_irq(SPIUART_SERIAL_VECTOR);
    
    /* enable UART and error/idle interrupts */
    SPIUART_rCR3 = USART_CR3_EIE;
    SPIUART_rCR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
    
#if 0	/* keep this for signal integrity testing */
    for (;;) {
        while (!(SPIUART_rSR & USART_SR_TXE))
            ;
        SPIUART_rDR = 0xfa;
        while (!(SPIUART_rSR & USART_SR_TXE))
            ;
        SPIUART_rDR = 0xa0;
    }
#endif
    
    /* configure RX DMA and return to listening state */
    spiuart_dma_reset();
    
    debug("spiuart init");

}

void spiuart_setbaud(uint32_t baudrate)
{
    /* reset and configure the UART */
    SPIUART_rCR1 = 0;
    SPIUART_rCR2 = 0;
    SPIUART_rCR3 = 0;
    
    /* clear status/errors */
    (void)SPIUART_rSR;
    (void)SPIUART_rDR;
    
    /* configure line speed */
    uint32_t usartdiv32 = 0;
    if (baudrate != 0) {
        usartdiv32 = SPIUART_SERIAL_CLOCK / (baudrate / 2);
    } else {
        usartdiv32 = SPIUART_SERIAL_CLOCK / (SPIUART_SERIAL_BITRATE / 2);
    }
    uint32_t mantissa = usartdiv32 >> 5;
    uint32_t fraction = (usartdiv32 - (mantissa << 5) + 1) >> 1;
    SPIUART_rBRR = (mantissa << USART_BRR_MANT_SHIFT) | (fraction << USART_BRR_FRAC_SHIFT);

    /* enable UART and error/idle interrupts */
    SPIUART_rCR3 = USART_CR3_EIE;
    SPIUART_rCR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
    
    /* configure RX DMA and return to listening state */
    spiuart_dma_reset();
}

int
spiuart_input(uint8_t *values, uint16_t num_values)
{
    uint32_t dma_rx_pt = SPIUART_RX_BUF_SIZE - stm32_dmaresidual(spiuart_rx_dma);
    uint32_t rx_len = 0;
    
    if (rxdmanext < dma_rx_pt) {
        
        rx_len = dma_rx_pt - rxdmanext;
        
        if (rx_len > num_values) rx_len = num_values;
        
        memcpy(&values[0], &spiuart_rx_buf[rxdmanext], rx_len);
        rxdmanext += rx_len;
        
    } else if (rxdmanext > dma_rx_pt) {
        
        rx_len = SPIUART_RX_BUF_SIZE + dma_rx_pt - rxdmanext;
        
        if (rx_len > num_values) rx_len = num_values;
        
        if (rx_len <= SPIUART_RX_BUF_SIZE - rxdmanext) {
            
            memcpy(&values[0], &spiuart_rx_buf[rxdmanext], rx_len);
            rxdmanext += rx_len;
            
        } else {
            
            memcpy(&values[0], &spiuart_rx_buf[rxdmanext], SPIUART_RX_BUF_SIZE - rxdmanext);
            memcpy(&values[SPIUART_RX_BUF_SIZE - rxdmanext], &spiuart_rx_buf[0], rx_len - (SPIUART_RX_BUF_SIZE - rxdmanext));
            rxdmanext = rxdmanext + rx_len - SPIUART_RX_BUF_SIZE;
            
        }
        
    } else {
        //buffer empty. do nothing
    }
    
    return rx_len;
}

int
spiuart_output(uint8_t *values, uint16_t num_values)
{
    if (num_values == 0) return 0;
    
    /* disable UART DMA */
    SPIUART_rCR3 &= ~USART_CR3_DMAT;
    
    /* kill any pending DMA */
    stm32_dmastop(spiuart_tx_dma);
    
    if (num_values <= SPIUART_TX_BUF_SIZE)
        memcpy(&spiuart_tx_buf[0], values, num_values);
    else
        memcpy(&spiuart_tx_buf[0], values, SPIUART_TX_BUF_SIZE);
    
    stm32_dmasetup(
                   spiuart_tx_dma,
                   (uint32_t)&SPIUART_rDR,
                   (uint32_t)&spiuart_tx_buf[0],
                   num_values,
                   DMA_CCR_DIR		|
                   DMA_CCR_MINC		|
                   DMA_CCR_PSIZE_8BITS	|
                   DMA_CCR_MSIZE_8BITS);
    stm32_dmastart(spiuart_tx_dma, NULL, NULL, false);
    
    SPIUART_rCR3 |= USART_CR3_DMAT;
    
    return num_values;
}

static int
serial_interrupt(int irq, void *context)
{
    uint32_t sr = SPIUART_rSR;	/* get UART status register */
    (void)SPIUART_rDR;		/* required to clear any of the interrupt status that brought us here */
    
    if (sr & USART_SR_ORE) {	/* overrun error - packet was too big for DMA or DMA was too slow */

        spiuart_dma_reset();
        return 0;
        
    }
    
    return 0;
}

static void
spiuart_dma_reset(void)
{
    SPIUART_rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
    (void)SPIUART_rSR;
    (void)SPIUART_rDR;
    (void)SPIUART_rDR;
    
    rxdmanext = 0;
    
    /* kill any pending DMA */
    stm32_dmastop(spiuart_tx_dma);
    stm32_dmastop(spiuart_rx_dma);
    
    /* reset the RX side */
    stm32_dmasetup(//
                   spiuart_rx_dma,
                   (uint32_t)&SPIUART_rDR,
                   (uint32_t)&spiuart_rx_buf[0],
                   SPIUART_RX_BUF_SIZE,
                   DMA_CCR_CIRC     |
                   DMA_CCR_MINC		|
                   DMA_CCR_PSIZE_8BITS	|
                   DMA_CCR_MSIZE_8BITS     |
                   DMA_CCR_PRIVERYHI);
    
    /* start receive DMA ready for the next packet */
    stm32_dmastart(spiuart_rx_dma, NULL, NULL, false);
    SPIUART_rCR3 |= USART_CR3_DMAR;
}