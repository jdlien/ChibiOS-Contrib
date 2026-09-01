/*
    ChibiOS - Copyright (C) 2023 1Conan

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_spi_v2_lld.c
 * @brief   SN32 SPI (v2) subsystem low level driver source.
 *
 * @addtogroup SPI_V2
 * @{
 */

#include "hal.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   SPI0 driver identifier.
 */
#if (SN32_SPI_USE_SPI0 == TRUE) || defined(__DOXYGEN__)
SPIDriver SPID0;
#endif

/**
 * @brief   SPI1 driver identifier.
 */
#if (SN32_SPI_USE_SPI1 == TRUE) || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/* SN32 SPI TX/RX FIFO depth (words). Both FIFOs are 8 entries deep. */
#define SN32_SPI_FIFO_DEPTH 8U

static void spi_lld_configure(SPIDriver *spip) {
  spip->spi->CTRL0 = spip->config->ctrl0;
  spip->spi->CTRL0_b.SELDIS = SPI_SELECT_MODE != SPI_SELECT_MODE_LLD;
  spip->spi->CTRL0_b.MS = spip->config->slave;
  spip->spi->CTRL0_b.SDODIS = false;
  /* Fire the RX threshold IRQ as soon as one word is available. The handler
     drains the whole FIFO per call, so a low threshold still yields few IRQs
     under load (bytes accumulate during IRQ latency) while guaranteeing the
     final short batch is always delivered -- no RX-timeout tail handling. */
  spip->spi->CTRL0_b.RXFIFOTH = 0;

  spip->spi->CTRL1 = (uint32_t)spip->config->ctrl1;

  spip->spi->CLKDIV = (uint32_t)spip->config->clkdiv;

  uint32_t sn32_spi_clock = (SN32_HCLK / ((2 * spip->config->clkdiv) + 2));
  if (sn32_spi_clock > 6000000) {
    spip->spi->DFDLY = true;
  }

  SPI_FIFO_FRESET(spip);

  spip->spi->IC = 0b1111;
  // enable RX FIFO threshold interrupt
  spip->spi->IE = 0b0100;

  spip->spi->CTRL0_b.SPIEN = true;
}

/* Top up the TX FIFO. Bytes are pushed while data remains, the TX FIFO has
   room, and no more than a FIFO's worth are outstanding (txidx - rxidx) -- the
   latter caps RX fill so it can never overflow, which would drop words and
   stall the rxidx==count completion test. Priming the FIFO (vs one byte per
   IRQ) keeps SCLK continuously clocked; that idle-gap removal is the main win. */
static inline void spi_fifo_fill(SPIDriver *spip) {
  while (spip->txidx < spip->count &&
         (spip->txidx - spip->rxidx) < SN32_SPI_FIFO_DEPTH &&
         !spip->spi->STAT_b.TX_FULL) {
    spip->spi->DATA = spip->txbuf ? spip->txbuf[spip->txidx] : 0x00;
    spip->txidx++;
  }
}

static inline void spi_lld_irq_handler(SPIDriver *spip) {
  if (spip->spi->RIS_b.RXFIFOTHIF || !spip->spi->STAT_b.RX_EMPTY) {
    /* Drain every word the RX FIFO accumulated since the last interrupt. */
    while (!spip->spi->STAT_b.RX_EMPTY) {
      uint16_t data = spip->spi->DATA;
      if (spip->rxbuf && spip->rxidx < spip->count) {
        spip->rxbuf[spip->rxidx] = data;
      }
      spip->rxidx++;
    }
    spip->spi->IC_b.RXFIFOTHIC = true;

    if (spip->rxidx >= spip->count) {
      __spi_isr_complete_code(spip);
    } else {
      /* Keep the pipeline full for the remainder of the transfer. */
      spi_fifo_fill(spip);
    }
  }
}

/*===========================================================================*/
/* SN32 SPI-to-SPI flash->LCD DMA extension.                                  */
/*===========================================================================*/
/* The SPI-to-SPI DMA registers live on SPI0 itself (DMACTRL/DMACNT/DMAHTCNT,
 * completion via SPI0's DMATCIF/DMAHTIF). They are absent from the minimal
 * sn32_spi.h view, so this uses the vendor SN_SPI0 map. Completion is dispatched
 * from SN32_SPI0_HANDLER below -- no application-provided vector. */
#if defined(SN32_SPI0_FLASH_DMA)

static spi_sn32_dma_cb_t sn32_dma_cb;
static volatile bool     sn32_dma_busy = false;
/* The flash-source SPI instance borrowed for the DMA (SPI1). Held for Step 2,
 * when SPID1 is spiStart()ed and we must keep its driver handler from running
 * during the transfer.
 *
 * IMPORTANT: do NOT mask the source's RXFIFOTHIE to do that -- on the SN32 the
 * RX-threshold event that RXFIFOTHIE enables is ALSO what triggers the SPI1->SPI0
 * DMA request. Clearing it stalls the DMA (it never fires, never completes, and
 * SPI0 is left in DMA mode -> the next spiSend hangs). The correct isolation,
 * once SPID1 is live, is nvicDisableVector(SN32_SPI1_NUMBER) for the DMA window
 * (keeps the threshold event -> keeps the DMA trigger, just skips the handler),
 * re-enabled on completion. Until SPID1 is started its vector is already off,
 * so this instance is currently only recorded, not touched. */
static SPIDriver        *sn32_dma_flash = NULL;

/* SPI0 in DMA-ready config: 8-bit words (command phase), mode 0, 24 MHz, data
 * fetch delay, re-latched by FRESET. Matches the hand-tuned bare-metal setup. */
static void sn32_flash_dma_config(void) {
  /* QP's spiStop() gates the SPI0 clock between flushes; re-enable it. */
  sys1EnableSPI0();
  /* Wipe the driver's leftover CTRL0/CTRL1 (spiStop does not clear them) so no
     stale DL/threshold/MDIV bits survive into the DMA config. */
  SN_SPI0->CTRL0 = 0;
  SN_SPI0->CTRL1 = 0;
  SN_SPI0->CTRL0_b.MS     = 0;
  SN_SPI0->CTRL0_b.SDODIS = 0;
  SN_SPI0->CTRL0_b.DL     = 7;       /* 8-bit */
  SN_SPI0->CTRL0_b.SELDIS = 1;
  SN_SPI0->CTRL1_b.MLSB   = 0;
  SN_SPI0->CTRL1_b.CPOL   = 0;       /* mode 0 */
  SN_SPI0->CTRL1_b.CPHA   = 0;
  SN_SPI0->CLKDIV_b.DIV   = 0;       /* 24 MHz */
  SN_SPI0->DFDLY_b.DFETCH_EN = 1;
  SN_SPI0->CTRL0_b.FRESET = 0b11;
  SN_SPI0->CTRL0_b.SPIEN  = 1;
}

void spiSN32FlashDmaPrepare(SPIDriver *lcd, SPIDriver *flash, uint32_t len) {
  (void)lcd;
  /* Borrow the source instance for the DMA window. Do NOT touch its RXFIFOTHIE:
     that bit gates the SPI1 RX-threshold event that triggers the DMA request, so
     clearing it would stall the transfer. Instead disable its NVIC vector -- the
     threshold event (and thus the DMA trigger) stays live, but the driver's
     SN32_SPI1_HANDLER won't dispatch and race the DMA draining the RX FIFO. The
     command phase (raw READ+addr the caller clocks next) also relies on the
     vector being off. Re-enabled in sn32_flash_dma_isr() on completion. */
  sn32_dma_flash = flash;
#if SN32_SPI_USE_SPI1 == TRUE
  if (flash == &SPID1) {
    nvicDisableVector(SN32_SPI1_NUMBER);
  }
#endif
  sn32_flash_dma_config();
  SN_SPI0->CTRL0_b.FRESET = 0b11;
  SN_SPI0->DMACTRL_b.DMAEN = 0;
  SN_SPI0->DMACTRL_b.DIR   = 0;              /* SPI1(flash) -> SPI0(LCD) */
  SN_SPI0->DMACNT_b.CNT    = len - 1;
  SN_SPI0->DMAHTCNT_b.HTCNT = (len - 1) / 2;
  /* SPI0 stays 8-bit here so the caller can send the panel window + flash
     READ+addr command before the pixel stream starts. */
}

void spiSN32FlashDmaFire(SPIDriver *lcd, spi_sn32_dma_cb_t cb) {
  (void)lcd;
  sn32_dma_cb   = cb;
  sn32_dma_busy = true;
  SN_SPI0->IC = 0x3F;
  SN_SPI0->CTRL0_b.DL = 0xF;                 /* 16-bit words (one RGB565 pixel) */
  SN_SPI0->IE = (1u << 5) | (1u << 4);       /* DMATCIE | DMAHTIE */
  nvicClearPending(SN32_SPI0_NUMBER);
  nvicEnableVector(SN32_SPI0_NUMBER, SN32_SPI_SPI0_IRQ_PRIORITY);
  SN_SPI0->DMACTRL_b.DMAEN = 1;              /* completion -> SN32_SPI0_HANDLER */
}

bool spiSN32FlashDmaBusy(SPIDriver *lcd) {
  (void)lcd;
  return sn32_dma_busy;
}

/* Abort an in-flight flash->LCD DMA and put BOTH controllers back exactly where
 * a normal completion would leave them.
 *
 * A caller that has waited out its bound cannot simply declare the blit done:
 * spiSN32FlashDmaPrepare() DISABLES SPI1's NVIC vector for the DMA window, and
 * only the completion path re-enables it. Tearing down without that leaves SPI1
 * permanently deaf, so the next flash read never completes and the board wedges
 * harder than the stall being recovered from -- observed on hardware
 * 2026-08-30, total silence within two seconds of an otherwise-successful
 * recovery.
 *
 * Order matters: stop the DMA engine BEFORE resetting the source FIFO, or the
 * sink is left armed waiting on data that will never arrive. */
void spiSN32FlashDmaAbort(SPIDriver *lcd) {
  (void)lcd;
  if (!sn32_dma_busy) {
    return;
  }
  SN_SPI0->DMACTRL_b.DMAEN = 0;              /* engine off first */
  SN_SPI0->IC = 0x3F;
  nvicClearPending(SN32_SPI0_NUMBER);
  SN_SPI0->CTRL0_b.DL = 7;                   /* back to 8-bit */
  /* Same hand-back to FIFO mode the completion path performs, or a subsequent
     spiSend on the panel would never complete. */
  SN_SPI0->CTRL0_b.RXFIFOTH = 0;
  SN_SPI0->IE = 0b0100;                      /* RXFIFOTHIE */
#if SN32_SPI_USE_SPI1 == TRUE
  if (sn32_dma_flash == &SPID1) {
    SN_SPI1->CTRL0_b.FRESET = 0b11;
    SN_SPI1->IC = 0x3F;
    nvicClearPending(SN32_SPI1_NUMBER);
    nvicEnableVector(SN32_SPI1_NUMBER, SN32_SPI_SPI1_IRQ_PRIORITY);
  }
#endif
  sn32_dma_flash = NULL;
  sn32_dma_busy  = false;
  sn32_dma_cb    = NULL;                     /* the caller does its own teardown */
}

/* Dispatched from the SPI0 handler when a DMA flag is pending. Returns true if
 * it consumed the interrupt (so the FIFO handler is skipped). */
static bool sn32_flash_dma_isr(void) {
  uint32_t ris = SN_SPI0->RIS;
  SN_SPI0->IC = 0x3F;
  if (ris & (1u << 5)) {                     /* DMATCIF: transfer complete */
    /* Let the last word finish shifting out before tearing down. */
    for (uint32_t g = 0; g < 200000u &&
         (!SN_SPI0->STAT_b.TX_EMPTY || SN_SPI0->STAT_b.BUSY); g++) { }
    SN_SPI0->DMACTRL_b.DMAEN = 0;
    SN_SPI0->CTRL0_b.DL = 7;                 /* back to 8-bit */
    /* Hand SPI0 back to the driver's FIFO mode: the DMA arm left the RX-threshold
       IRQ disabled (IE = DMA bits), so a subsequent spiSend would never complete.
       QP-driven callers re-run spiStart per flush and don't need this; callers
       that drive the driver directly between DMAs (e.g. a bare-metal dashboard)
       do. Restoring the driver's IE here is harmless for the former. */
    SN_SPI0->CTRL0_b.RXFIFOTH = 0;
    SN_SPI0->IE = 0b0100;                    /* RXFIFOTHIE */
#if SN32_SPI_USE_SPI1 == TRUE
    if (sn32_dma_flash == &SPID1) {
      /* Flush any stale RX + pending flag so handing the vector back doesn't
         spuriously dispatch the idle driver handler, then re-enable it. */
      SN_SPI1->CTRL0_b.FRESET = 0b11;
      SN_SPI1->IC = 0x3F;
      nvicClearPending(SN32_SPI1_NUMBER);
      nvicEnableVector(SN32_SPI1_NUMBER, SN32_SPI_SPI1_IRQ_PRIORITY);
    }
#endif
    sn32_dma_flash = NULL;
    sn32_dma_busy = false;
    if (sn32_dma_cb) sn32_dma_cb();          /* caller drops flash/panel CS */
  }
  return true;                               /* DMAHTIF: consumed, nothing to do */
}

#endif /* SN32_SPI0_FLASH_DMA */

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SN32_SPI_USE_SPI0
OSAL_IRQ_HANDLER(SN32_SPI0_HANDLER) {
  OSAL_IRQ_PROLOGUE();

#if defined(SN32_SPI0_FLASH_DMA)
  /* Service a pending flash->LCD DMA completion; else fall through to FIFO. */
  if (!((SPID0.spi->RIS & 0x30U) && sn32_flash_dma_isr()))  /* DMAHTIF|DMATCIF */
#endif
    spi_lld_irq_handler(&SPID0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SN32_SPI_USE_SPI1
OSAL_IRQ_HANDLER(SN32_SPI1_HANDLER) {
  OSAL_IRQ_PROLOGUE();

  spi_lld_irq_handler(&SPID1);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {

#if SN32_SPI_USE_SPI0 == TRUE
  /* Driver initialization.*/
  spiObjectInit(&SPID0);
  SPID0.spi = SN32_SPI0;
#endif

#if SN32_SPI_USE_SPI1 == TRUE
  /* Driver initialization.*/
  spiObjectInit(&SPID1);
  SPID1.spi = SN32_SPI1;
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_start(SPIDriver *spip) {

  if (spip->state == SPI_STOP) {

    /* Enables the peripheral.*/

#if SN32_SPI_USE_SPI0 == TRUE
    if (&SPID0 == spip) {
      sys1EnableSPI0();
      nvicClearPending(SN32_SPI0_NUMBER);
      nvicEnableVector(SN32_SPI0_NUMBER, SN32_SPI_SPI0_IRQ_PRIORITY);
    }
#endif

#if SN32_SPI_USE_SPI1 == TRUE
    if (&SPID1 == spip) {
      sys1EnableSPI1();
      nvicClearPending(SN32_SPI1_NUMBER);
      nvicEnableVector(SN32_SPI1_NUMBER, SN32_SPI_SPI1_IRQ_PRIORITY);
    }
#endif

    else {
      osalDbgAssert(false, "invalid SPI instance");
    }
  }

  spi_lld_configure(spip);

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  if (spip->state == SPI_READY) {

    /* Disables the peripheral.*/

    SPI_FIFO_FRESET(spip);

    spip->spi->CTRL0_b.SPIEN = false;

#if SN32_SPI_USE_SPI0 == TRUE
    if (&SPID0 == spip) {
      sys1DisableSPI0();
      nvicDisableVector(SN32_SPI0_NUMBER);
    }
#endif

#if SN32_SPI_USE_SPI1 == TRUE
    if (&SPID1 == spip) {
      sys1DisableSPI1();
      nvicDisableVector(SN32_SPI1_NUMBER);
    }
#endif

    else {
      osalDbgAssert(false, "invalid SPI instance");
    }
  }
}

#if (SPI_SELECT_MODE == SPI_SELECT_MODE_LLD) || defined(__DOXYGEN__)
/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {

  (void)spip;
  // noop - hardware auto-sel
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip) {

  (void)spip;
  // noop - hardware auto-sel
}
#endif

/**
 * @brief   Ignores data on the SPI bus.
 * @details This synchronous function performs the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @pre     In order to use this function the option @p SPI_USE_SYNCHRONIZATION
 *          must be enabled.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_ignore(SPIDriver *spip, size_t n) {
  return spi_lld_exchange(spip, n, NULL, NULL);
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_exchange(SPIDriver *spip, size_t n,
                       const void *txbuf, void *rxbuf) {

  spip->txbuf = txbuf;
  spip->rxbuf = rxbuf;
  spip->count = n;
  spip->rxidx = 0;
  spip->txidx = 0;

  /* Prime the TX FIFO so SCLK runs continuously; the RX threshold IRQ then
     drains completed words and refills. */
  spi_fifo_fill(spip);

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {
  return spi_lld_exchange(spip, n, txbuf, NULL);
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {
  return spi_lld_exchange(spip, n, NULL, rxbuf);
}

/**
 * @brief   Aborts the ongoing SPI operation, if any.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[out] sizep    pointer to the counter of frames not yet transferred
 *                      or @p NULL
 * @return              The operation status.
 *
 * @notapi
 */
msg_t spi_lld_stop_transfer(SPIDriver *spip, size_t *sizep) {

  SPI_FIFO_FRESET(spip);

  if (sizep != NULL) {
    *sizep = spip->count - spip->rxidx;
  }

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {
  spip->spi->DATA = frame;
  while (spip->spi->STAT_b.RX_EMPTY);

  return spip->spi->DATA;
}

#endif /* HAL_USE_SPI */

/** @} */
