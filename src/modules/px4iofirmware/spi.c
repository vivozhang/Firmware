/****************************************************************************
 *
 *   Copyright (C) 2012,2013 PX4 Development Team. All rights reserved.
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
 * @file spi.c
 *
 * SPI communication for the raspilotio module.
 */

#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <stm32_spi.h>
#include <stm32_dma.h>

/* XXX might be able to prune these */
#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>
#include <stm32.h>
#include <systemlib/perf_counter.h>

//#define DEBUG
#include "px4io.h"

static perf_counter_t	pc_txns;
static perf_counter_t	pc_errors;
static perf_counter_t	pc_ore;
static perf_counter_t	pc_fe;
static perf_counter_t	pc_ne;
static perf_counter_t	pc_idle;
static perf_counter_t	pc_badidle;
static perf_counter_t	pc_regerr;
static perf_counter_t	pc_crcerr;

/*
 * SPI register definitions.
 */
#define SPI_BASE	STM32_SPI3_BASE

#define REG(_reg)	(*(volatile uint32_t *)(SPI_BASE + _reg))

#define rCR1		REG(STM32_SPI_CR1_OFFSET)
#define rCR2		REG(STM32_SPI_CR2_OFFSET)
#define rSR		    REG(STM32_SPI_SR_OFFSET)
#define rDR		    REG(STM32_SPI_DR_OFFSET)
#define rCRCPR		REG(STM32_SPI_CRCPR_OFFSET)
#define rRXCRCR		REG(STM32_SPI_RXCRCR_OFFSET)
#define rTXCRCR		REG(STM32_SPI_TXCRCR_OFFSET)

static struct IOPacket	dma_packet;

void			spi_reset(void);

static int		spi_interrupt(int irq, void *context);
static void		dma_reset(void);

static void		rx_handle_packet(void);
static void		rx_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);

#ifdef DEBUG
static void		spi_dump(void);
#endif

static DMA_HANDLE	rx_dma;
static DMA_HANDLE	tx_dma;

void
interface_init(void)
{
    debug("spi init");
    
    pc_txns = perf_alloc(PC_ELAPSED, "txns");
    pc_errors = perf_alloc(PC_COUNT, "errors");
    pc_ore = perf_alloc(PC_COUNT, "overrun");
    pc_fe = perf_alloc(PC_COUNT, "framing");
    pc_ne = perf_alloc(PC_COUNT, "noise");
    pc_idle = perf_alloc(PC_COUNT, "idle");
    pc_badidle = perf_alloc(PC_COUNT, "badidle");
    pc_regerr = perf_alloc(PC_COUNT, "regerr");
    pc_crcerr = perf_alloc(PC_COUNT, "crcerr");

    /* allocate DMA handles and initialise DMA */
    rx_dma = stm32_dmachannel(DMACHAN_SPI3_RX);
    tx_dma = stm32_dmachannel(DMACHAN_SPI3_TX);
    
    /* enable the spi block clock and reset it */
    modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_SPI3EN);
    modifyreg32(STM32_RCC_APB1RSTR, 0, RCC_APB1RSTR_SPI3RST);
    modifyreg32(STM32_RCC_APB1RSTR, RCC_APB1RSTR_SPI3RST, 0);
    
    /* configure the spi GPIOs */
    stm32_configgpio(RPI_SPI_NSS);
    stm32_configgpio(RPI_SPI_SCK);
    stm32_configgpio(RPI_SPI_MISO);
    stm32_configgpio(RPI_SPI_MOSI);
    
    /* reset and configure the SPI */
    rCR1 = SPI_CR1_CPHA | SPI_CR1_CPOL;
    
    /* set for DMA operation */
    rCR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_ERRIE;
    
    /* enable event interrupts */
    irq_attach(STM32_IRQ_SPI3, spi_interrupt);
    up_enable_irq(STM32_IRQ_SPI3);
    
    /* configure RX DMA and return to listening state */
    dma_reset();
    
    /* and enable the SPI port */
    rCR1 |= SPI_CR1_SPE;

#ifdef DEBUG
	spi_dump();
#endif
}

/*
  reset the SPI bus
  used to recover from lockups
 */
void
spi_reset(void)
{
    /* reset and configure the SPI */
    rCR1 = SPI_CR1_CPHA | SPI_CR1_CPOL;
    
    /* set for DMA operation */
    rCR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_ERRIE;
    
    /* and enable the SPI port */
    rCR1 |= SPI_CR1_SPE;

}

static void
rx_handle_packet(void)
{
    /* check packet CRC */
    uint8_t crc = dma_packet.crc;
    dma_packet.crc = 0;
    if (crc != crc_packet(&dma_packet)) {
        perf_count(pc_crcerr);
        
        /* send a CRC error reply */
        dma_packet.count_code = PKT_CODE_CORRUPT;
        dma_packet.page = 0xff;
        dma_packet.offset = 0xff;
        
        return;
    }
    
    if (PKT_CODE(dma_packet) == PKT_CODE_WRITE) {
        
        /* it's a blind write - pass it on */
        if (registers_set(dma_packet.page, dma_packet.offset, &dma_packet.regs[0], PKT_COUNT(dma_packet))) {
            perf_count(pc_regerr);
            dma_packet.count_code = PKT_CODE_ERROR;
        } else {
            dma_packet.count_code = PKT_CODE_SUCCESS;
        }
        return;
    }
    
    if (PKT_CODE(dma_packet) == PKT_CODE_READ) {
        
        /* it's a read - get register pointer for reply */
        unsigned count;
        uint16_t *registers;
        
        if (registers_get(dma_packet.page, dma_packet.offset, &registers, &count) < 0) {
            perf_count(pc_regerr);
            dma_packet.count_code = PKT_CODE_ERROR;
        } else {
            /* constrain reply to requested size */
            if (count > PKT_MAX_REGS)
                count = PKT_MAX_REGS;
            if (count > PKT_COUNT(dma_packet))
                count = PKT_COUNT(dma_packet);
            
            /* copy reply registers into DMA buffer */
            memcpy((void *)&dma_packet.regs[0], registers, count * 2);
            dma_packet.count_code = count | PKT_CODE_SUCCESS;
        }
        return;
    }
    
    if (PKT_CODE(dma_packet) == PKT_CODE_SPIUART) {
        
        /* it's a read - get register pointer for reply */
        unsigned count;
        uint16_t *registers;
        
        if (registers_spiuart(dma_packet.page, dma_packet.offset, &dma_packet.regs[0], &registers, &count) < 0) {
            perf_count(pc_regerr);
            dma_packet.count_code = PKT_CODE_ERROR;
            dma_packet.offset = 0;
        } else {
            /* copy reply registers into DMA buffer */
            memcpy((void *)&dma_packet.regs[0], registers, count);
            dma_packet.offset = count;
            dma_packet.count_code = PKT_MAX_REGS | PKT_CODE_SUCCESS;
        }
        return;
    }
    
    /* send a bad-packet error reply */
    dma_packet.count_code = PKT_CODE_CORRUPT;
    dma_packet.page = 0xff;
    dma_packet.offset = 0xfe;
}

static void
rx_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
    uint16_t sr = rSR;
    
    stm32_dmastop(rx_dma);
    stm32_dmastop(tx_dma);
    
    /* handle the received packet */
    rx_handle_packet();
    
    /* re-set DMA for reception first, so we are ready to receive before we start sending */
    if (!(sr & SPI_SR_BSY)) {
        dma_reset();
    }
    
    /* send the reply to the just-processed request */
    dma_packet.crc = 0;
    dma_packet.crc = crc_packet(&dma_packet);
    stm32_dmasetup(
                   tx_dma,
                   (uint32_t)&rDR,
                   (uint32_t)&dma_packet,
                   PKT_SIZE(dma_packet),
                   DMA_CCR_DIR		|
                   DMA_CCR_MINC		|
                   DMA_CCR_PSIZE_8BITS	|
                   DMA_CCR_MSIZE_8BITS	|
                   DMA_CCR_PRIMED);
    stm32_dmastart(tx_dma, NULL, NULL, false);
    
    perf_end(pc_txns);
}

static int
spi_interrupt(int irq, FAR void *context)
{
	uint16_t sr = rSR;

    if (sr & SPI_SR_OVR)
    {
        (void)rSR;
        rSR = 0;
        
        if (sr & SPI_SR_BSY) {
            stm32_dmastop(rx_dma);
        } else {
            dma_reset();
        }
    }

	/* clear any errors that might need it */
	if ((sr & SPI_SR_CRCERR) || (sr & SPI_SR_MODF))
    {
        (void)rSR;
        rSR = 0;
    }

	return 0;
}

static void
dma_reset(void)
{
    /* kill any pending DMA */
    stm32_dmastop(tx_dma);
    stm32_dmastop(rx_dma);
    
    /* reset the RX side */
    stm32_dmasetup(
                   rx_dma,
                   (uint32_t)&rDR,
                   (uint32_t)&dma_packet,
                   sizeof(dma_packet),
                   DMA_CCR_MINC		|
                   DMA_CCR_PSIZE_8BITS	|
                   DMA_CCR_MSIZE_8BITS     |
                   DMA_CCR_PRIMED);
    
    /* start receive DMA ready for the next packet */
    stm32_dmastart(rx_dma, rx_dma_callback, NULL, false);
}

#ifdef DEBUG
static void
spi_dump(void)
{
	debug("CR1   0x%08x  CR2   0x%08x", rCR1,  rCR2);
	debug("SR    0x%08x", rSR);
}
#endif
