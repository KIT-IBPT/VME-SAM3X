/**
 * \file
 *
 * \brief Two-Wire Interface (TWI) driver for SAM.
 *
 * Copyright (c) 2011-2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "twi.h"

#ifdef FREERTOS_USED
#	include "FreeRTOS.h"
#	include "task.h"
#endif // FREERTOS_USED

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \defgroup sam_drivers_twi_group Two-Wire Interface (TWI)
 *
 * Driver for the TWI (Two-Wire Interface). This driver provides access to the main 
 * features of the TWI controller.
 * The TWI interconnects components on a unique two-wire bus.
 * The TWI is programmable as a master or a slave with sequential or single-byte access.
 * Multiple master capability is supported.
 *
 * \par Usage
 *
 * -# Enable the TWI peripheral clock in the PMC.
 * -# Enable the required TWI PIOs (see pio.h).
 * -# Enable TWI master mode by calling twi_enable_master_mode if it is a master on the I2C bus.
 * -# Configure the TWI in master mode by calling twi_master_init.
 * -# Send data to a slave device on the I2C bus by calling twi_master_write.
 * -# Receive data from a slave device on the I2C bus by calling the twi_master_read.
 * -# Enable TWI slave mode by calling twi_enable_slave_mode if it is a slave on the I2C bus.
 * -# Configure the TWI in slave mode by calling twi_slave_init.
 *
 * @{
 */

#define I2C_FAST_MODE_SPEED  400000
#define TWI_CLK_DIVIDER      2
#define TWI_CLK_CALC_ARGU    4
#define TWI_CLK_DIV_MAX      0xFF
#define TWI_CLK_DIV_MIN      7

#define TWI_WP_KEY_VALUE TWI_WPMR_WPKEY_PASSWD

#ifdef FREERTOS_USED
static TaskHandle_t twi0_task;
static TaskHandle_t twi1_task;

static void twi_interrupt_handler(Twi *p_twi, TaskHandle_t task)
{
	BaseType_t higher_priority_task_woken = pdFALSE;
	/* When we receive any interrupt, we disable the full interrupt mask. We
	can do this without reading the status register, and reading the status
	register may have unintended side effects (some flags are cleared on
	read). */
	twi_disable_interrupt(p_twi, 0x0000FF77);
	xTaskNotifyFromISR(task, 0, eNoAction, &higher_priority_task_woken);
	portYIELD_FROM_ISR(higher_priority_task_woken);
}

static uint32_t twi_set_task_handle(Twi *p_twi, TaskHandle_t current_task)
{
	if (p_twi == TWI0) {
		twi0_task = current_task;
		return TWI_SUCCESS;
	} else if (p_twi == TWI1) {
		twi1_task = current_task;
		return TWI_SUCCESS;
	} else {
		return TWI_INVALID_ARGUMENT;
	}
}

static uint32_t twi_wait_for_interrupt(
		Twi *p_twi, uint32_t flags, TickType_t ticks_to_wait)
{
	TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
	uint32_t rc = TWI_SUCCESS;

	rc = twi_set_task_handle(p_twi, current_task);
	if (rc != TWI_SUCCESS) {
		return rc;
	}
	/* We are only interested in task notifications that happen after we have
	enabled the interrupt. */
	xTaskNotifyStateClear(current_task);
	twi_enable_interrupt(p_twi, flags);
	/* Wait for the interrupt handler to be called. The time needed to transfer
	a byte over TWI is short compared to the length of a tick, so waiting for a
	single tick should be sufficient. However, experiments have shown that
	timeouts are extremely frequent when speccifying a timeout of one. This
	might be due to how the scheduling logic of FreeRTOS works. With a timeout
	of 2, this problem does not appear. */
	if (xTaskNotifyWait(0, 0, NULL, ticks_to_wait) == pdFALSE) {
		/* Usually, interrupts are disabled by the interrupt handler, but in
		case of a timeout, we have to disable them here. */
		twi_disable_interrupt(p_twi, flags);
		rc = TWI_ERROR_TIMEOUT;
	}
	return rc;
}

void TWI0_Handler(void)
{
	twi_interrupt_handler(TWI0, twi0_task);
}

void TWI1_Handler(void)
{
	twi_interrupt_handler(TWI1, twi1_task);
}
#endif // FREERTOS_USED

/**
 * \brief Enable TWI master mode.
 *
 * \param p_twi Pointer to a TWI instance.
 */
void twi_enable_master_mode(Twi *p_twi)
{
	/* Set Master Disable bit and Slave Disable bit */
	p_twi->TWI_CR = TWI_CR_MSDIS;
	p_twi->TWI_CR = TWI_CR_SVDIS;

	/* Set Master Enable bit */
	p_twi->TWI_CR = TWI_CR_MSEN;
}

/**
 * \brief Disable TWI master mode.
 *
 * \param p_twi Pointer to a TWI instance.
 */
void twi_disable_master_mode(Twi *p_twi)
{
	/* Set Master Disable bit */
	p_twi->TWI_CR = TWI_CR_MSDIS;
}

/**
 * \brief Initialize TWI master mode.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param p_opt Options for initializing the TWI module (see \ref twi_options_t).
 *
 * \return TWI_SUCCESS if initialization is complete, error code otherwise.
 */
uint32_t twi_master_init(Twi *p_twi, const twi_options_t *p_opt)
{
	uint32_t status = TWI_SUCCESS;

#ifdef FREERTOS_USED
	enum IRQn twi_irq;
	if (p_twi == TWI0) {
		twi_irq = TWI0_IRQn;
	} else if (p_twi == TWI1) {
		twi_irq = TWI1_IRQn;
	} else {
		return TWI_INVALID_ARGUMENT;
	}
#endif // FREERTOS_USED

	/* Disable TWI interrupts */
	p_twi->TWI_IDR = ~0UL;

	/* Dummy read in status register */
	p_twi->TWI_SR;

#ifdef FREERTOS_USED
	/* The interrupt priority  must not be higher (numerically less than)
	configMAX_SYSCALL_INTERRUPT_PRIORITY. The lowest priority is
	configKERNEL_INTERRUPT_PRIORITY. Both constants are already left-shifted,
	but NVIC_SetPriority also applies left-shifting, so we have to unshift
	them. Without left-shifting, the max. interrupt priority for syscalls is 5
	and the kernel priority is 15. Again, remember that for the ARM Cortex M3,
	higher values mean a lower priority (the highest priority is zero). */
	NVIC_SetPriority(twi_irq, configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4);
	NVIC_EnableIRQ(twi_irq);
#endif // FREERTOS_USED

	/* Reset TWI peripheral */
	twi_reset(p_twi);

	twi_enable_master_mode(p_twi);

	/* Select the speed */
	if (twi_set_speed(p_twi, p_opt->speed, p_opt->master_clk) == FAIL) {
		/* The desired speed setting is rejected */
		status = TWI_INVALID_ARGUMENT;
	}

	if (p_opt->smbus == 1) {
		p_twi->TWI_CR = TWI_CR_QUICK;
	}

	return status;
}

/**
 * \brief Set the I2C bus speed in conjunction with the clock frequency.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param ul_speed The desired I2C bus speed (in Hz).
 * \param ul_mck Main clock of the device (in Hz).
 *
 * \retval PASS New speed setting is accepted.
 * \retval FAIL New speed setting is rejected.
 */
uint32_t twi_set_speed(Twi *p_twi, uint32_t ul_speed, uint32_t ul_mck)
{
	uint32_t ckdiv = 0;
	uint32_t c_lh_div;

	if (ul_speed > I2C_FAST_MODE_SPEED) {
		return FAIL;
	}

	c_lh_div = ul_mck / (ul_speed * TWI_CLK_DIVIDER) - TWI_CLK_CALC_ARGU;

	/* cldiv must fit in 8 bits, ckdiv must fit in 3 bits */
	while ((c_lh_div > TWI_CLK_DIV_MAX) && (ckdiv < TWI_CLK_DIV_MIN)) {
		/* Increase clock divider */
		ckdiv++;
		/* Divide cldiv value */
		c_lh_div /= TWI_CLK_DIVIDER;
	}

	/* set clock waveform generator register */
	p_twi->TWI_CWGR =
			TWI_CWGR_CLDIV(c_lh_div) | TWI_CWGR_CHDIV(c_lh_div) |
			TWI_CWGR_CKDIV(ckdiv);

	return PASS;
}

/**
 * \brief Test if a chip answers a given I2C address.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param uc_slave_addr Address of the remote chip to search for.
 *
 * \return TWI_SUCCESS if a chip was found, error code otherwise.
 */
uint32_t twi_probe(Twi *p_twi, uint8_t uc_slave_addr)
{
	twi_packet_t packet;
	uint8_t data = 0;

	/* Data to send */
	packet.buffer = &data;
	/* Data length */
	packet.length = 1;
	/* Slave chip address */
	packet.chip = (uint32_t) uc_slave_addr;
	/* Internal chip address */
	packet.addr[0] = 0;
	/* Address length */
	packet.addr_length = 0;

	/* Perform a master write access */
	return (twi_master_write(p_twi, &packet));
}


/**
 * \internal
 * \brief Construct the TWI module address register field
 *
 * The TWI module address register is sent out MSB first. And the size controls
 * which byte is the MSB to start with.
 *
 * Please see the device datasheet for details on this.
 */
static uint32_t twi_mk_addr(const uint8_t *addr, int len)
{
	uint32_t val;

	if (len == 0)
		return 0;

	val = addr[0];
	if (len > 1) {
		val <<= 8;
		val |= addr[1];
	}
	if (len > 2) {
		val <<= 8;
		val |= addr[2];
	}
	return val;
}

/**
 * \brief Read multiple bytes from a TWI compatible slave device.
 *
 * \note This function will NOT return until all data has been read or error occurs.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param p_packet Packet information and data (see \ref twi_packet_t).
 *
 * \return TWI_SUCCESS if all bytes were read, error code otherwise.
 */
uint32_t twi_master_read(Twi *p_twi, twi_packet_t *p_packet)
{
	uint32_t status;
	uint32_t cnt = p_packet->length;
	uint8_t *buffer = p_packet->buffer;
	uint8_t stop_sent = 0;
	uint32_t timeout = TWI_TIMEOUT;;

	/* Check argument */
	if (cnt == 0) {
		return TWI_INVALID_ARGUMENT;
	}

	/* Set read mode, slave address and 3 internal address byte lengths */
	p_twi->TWI_MMR = 0;
	p_twi->TWI_MMR = TWI_MMR_MREAD | TWI_MMR_DADR(p_packet->chip) |
			((p_packet->addr_length << TWI_MMR_IADRSZ_Pos) &
			TWI_MMR_IADRSZ_Msk);

	/* Set internal address for remote chip */
	p_twi->TWI_IADR = 0;
	p_twi->TWI_IADR = twi_mk_addr(p_packet->addr, p_packet->addr_length);

	/* Send a START condition */
	if (cnt == 1) {
		p_twi->TWI_CR = TWI_CR_START | TWI_CR_STOP;
		stop_sent = 1;
	} else {
		p_twi->TWI_CR = TWI_CR_START;
		stop_sent = 0;
	}

	/* If there is more than one byte to be received, use DMA to receive all
	but the last byte. Before receiving the last byte, we have to set the stop
	bit. */
	while (cnt > 1) {
		uint32_t rc = TWI_SUCCESS;
		Pdc *p_pdc = twi_get_pdc_base(p_twi);
		uint32_t remaining;
		uint32_t transfer_size;
		if (cnt > 0xFFFF) {
			transfer_size = 0xFFFF;
		} else {
			transfer_size = cnt - 1;
		}
		remaining = transfer_size;
		p_pdc->PERIPH_RPR = (uint32_t) buffer;
		p_pdc->PERIPH_RCR = transfer_size;
		p_pdc->PERIPH_PTCR = PERIPH_PTCR_RXTEN;
		timeout = TWI_TIMEOUT;
		while (1) {
			status = p_twi->TWI_SR;
			if (status & TWI_SR_NACK) {
				rc = TWI_RECEIVE_NACK;
				break;
			}
			if (status & TWI_SR_RXBUFF) {
				break;
			}
#ifdef FREERTOS_USED
			/* The timeout value of 20 ticks has been determined
			experimentally. It is about the smallest value at which there are
			no spurious timeouts. */
			rc = twi_wait_for_interrupt(p_twi, TWI_SR_RXBUFF, 20);
			if (rc == TWI_ERROR_TIMEOUT && p_pdc->PERIPH_RCR != remaining) {
				/* If there was a timeout but the transfer has progressed,
				there simply might be a lot of data to be transferred, so we
				continue. */
#	ifdef DEBUG_TWI
				printf("twi_master_read DMA transfer still in progress.\r\n");
#	endif // DEBUG_TWI
				remaining = p_pdc->PERIPH_RCR;
				continue;
			}
			if (rc != TWI_SUCCESS) {
#	ifdef DEBUG_TWI
				if (rc == TWI_ERROR_TIMEOUT) {
					printf(
							"twi_master_read timed out while waiting for DMA "
							"transfer.\r\n");
				} else {
					printf(
							"twi_master_read encountered error while waiting "
							"for DMA transfer.\r\n");
				}
#	endif // DEBUG_TWI
				break;
			}
#else // FREERTOS_USED
			if (!timeout--) {
				if (p_pdc->PERIPH_RCR != remaining) {
					remaining = p_pdc->PERIPH_RCR;
					timeout = TWI_TIMEOUT;
#	ifdef DEBUG_TWI
					printf(
							"twi_master_read DMA transfer still in "
							"progress.\r\n");
#	endif // DEBUG_TWI
					continue;
				}
				rc = TWI_ERROR_TIMEOUT;
#	ifdef DEBUG_TWI
				printf(
						"twi_master_read timed out while waiting for DMA "
						"transfer.\r\n");
#	endif // DEBUG_TWI
				break;
			}
#endif // FREERTOS_USED
		}
		p_pdc->PERIPH_PTCR = PERIPH_PTCR_RXTDIS;
		if (rc != TWI_SUCCESS) {
			p_pdc->PERIPH_RCR = 0;
			return rc;
		}
		buffer += transfer_size;
		cnt -= transfer_size;
	}

	/* Receive the last byte */
	if (!stop_sent) {
		p_twi->TWI_CR = TWI_CR_STOP;
		stop_sent = 1;
	}
	while (cnt) {
		status = p_twi->TWI_SR;
		if (status & TWI_SR_NACK) {
			return TWI_RECEIVE_NACK;
		}

		if (!timeout--) {
#	ifdef DEBUG_TWI
			printf("twi_master_read timed out while waiting for RXRDY.\r\n");
#	endif // DEBUG_TWI
			return TWI_ERROR_TIMEOUT;
		}
				
		if (!(status & TWI_SR_RXRDY)) {
#ifdef FREERTOS_USED
			uint32_t rc = twi_wait_for_interrupt(
					p_twi, TWI_SR_NACK | TWI_SR_RXRDY, 2);
			if (rc != TWI_SUCCESS) {
#	ifdef DEBUG_TWI
			if (rc == TWI_ERROR_TIMEOUT) {
				printf(
						"twi_master_read timed out while waiting for "
						"RXRDY.\r\n");
			} else {
				printf(
						"twi_master_read error occurred while waiting for "
						"RXRDY.\r\n");
			}
#	endif // DEBUG_TWI
				return rc;
			}
#endif // FREERTOS_USED
			continue;
		}
		*buffer++ = p_twi->TWI_RHR;

		cnt--;
		timeout = TWI_TIMEOUT;
	}

	timeout = TWI_TIMEOUT;
	while (!(p_twi->TWI_SR & TWI_SR_TXCOMP)) {
#ifdef FREERTOS_USED
		/* While timeouts occasionally happen here, experiments have shown that
		increasing the timout does not reduce the number of these events, so we
		use a rather short one. */
		uint32_t rc = twi_wait_for_interrupt(p_twi, TWI_SR_TXCOMP, 1);
		if (rc == TWI_ERROR_TIMEOUT) {
#	ifdef DEBUG_TWI
			printf("twi_master_read timed out while waiting for TXCOMP.\r\n");
#	endif // DEBUG_TWI
			return TWI_ERROR_TIMEOUT_COMP;
		} else if (rc != TWI_SUCCESS) {
			return rc;
		}
#endif // FREERTOS_USED
		if (!timeout--) {
#ifdef DEBUG_TWI
			printf("twi_master_read timed out while waiting for TXCOMP.\r\n");
#endif // DEBUG_TWI
			return TWI_ERROR_TIMEOUT_COMP;
		}
	}

	return TWI_SUCCESS;
}

/**
 * \brief Write multiple bytes to a TWI compatible slave device.
 *
 * \note This function will NOT return until all data has been written or error occurred.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param p_packet Packet information and data (see \ref twi_packet_t).
 *
 * \return TWI_SUCCESS if all bytes were written, error code otherwise.
 */
uint32_t twi_master_write(Twi *p_twi, twi_packet_t *p_packet)
{
	uint32_t status;
	uint32_t cnt = p_packet->length;
	uint8_t *buffer = p_packet->buffer;
	uint32_t timeout = TWI_TIMEOUT;

	/* Check argument */
	if (cnt == 0) {
		return TWI_INVALID_ARGUMENT;
	}

	/* Set write mode, slave address and 3 internal address byte lengths */
	p_twi->TWI_MMR = 0;
	p_twi->TWI_MMR = TWI_MMR_DADR(p_packet->chip) |
			((p_packet->addr_length << TWI_MMR_IADRSZ_Pos) &
			TWI_MMR_IADRSZ_Msk);

	/* Set internal address for remote chip */
	p_twi->TWI_IADR = 0;
	p_twi->TWI_IADR = twi_mk_addr(p_packet->addr, p_packet->addr_length);

	/* If there is more than one byte to be sent, use DMA to send all bytes */
	while (cnt > 1) {
		uint32_t rc = TWI_SUCCESS;
		Pdc *p_pdc = twi_get_pdc_base(p_twi);
		uint32_t remaining;
		uint32_t transfer_size;
		if (cnt > 0xFFFF) {
			transfer_size = 0xFFFF;
		} else {
			transfer_size = cnt;
		}
		remaining = transfer_size;
		p_pdc->PERIPH_TPR = (uint32_t) buffer;
		p_pdc->PERIPH_TCR = transfer_size;
		p_pdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;
		timeout = TWI_TIMEOUT;
		while (1) {
			status = p_twi->TWI_SR;
			if (status & TWI_SR_NACK) {
				rc = TWI_RECEIVE_NACK;
				break;
			}
			if (status & TWI_SR_TXBUFE) {
				break;
			}
#ifdef FREERTOS_USED
			/* The timeout value of 20 ticks has been determined
			experimentally. It is about the smallest value at which there are
			no spurious timeouts. */
			rc = twi_wait_for_interrupt(p_twi, TWI_SR_TXBUFE, 20);
			if (rc == TWI_ERROR_TIMEOUT && p_pdc->PERIPH_TCR != remaining) {
				/* If there was a timeout but the transfer has progressed,
				there simply might be a lot of data to be transferred, so we
				continue. */
#	ifdef DEBUG_TWI
				printf("twi_master_write DMA transfer still in progress.\r\n");
#	endif // DEBUG_TWI
				remaining = p_pdc->PERIPH_TCR;
				continue;
			}
			if (rc != TWI_SUCCESS) {
#	ifdef DEBUG_TWI
				if (rc == TWI_ERROR_TIMEOUT) {
					printf(
							"twi_master_write timed out while waiting for DMA "
							"transfer.\r\n");
				} else {
					printf(
							"twi_master_write encountered error while waiting "
							"for DMA transfer.\r\n");
				}
#	endif // DEBUG_TWI
				break;
			}
#else // FREERTOS_USED
			if (!timeout--) {
				if (p_pdc->PERIPH_TCR != remaining) {
					remaining = p_pdc->PERIPH_TCR;
					timeout = TWI_TIMEOUT;
#	ifdef DEBUG_TWI
					printf(
							"twi_master_write DMA transfer still in "
							"progress.\r\n");
#	endif // DEBUG_TWI
					continue;
				}
				rc = TWI_ERROR_TIMEOUT;
#	ifdef DEBUG_TWI
				printf(
						"twi_master_write timed out while waiting for DMA "
						"transfer.\r\n");
#	endif // DEBUG_TWI
				break;
			}
#endif // FREERTOS_USED
		}
		p_pdc->PERIPH_PTCR = PERIPH_PTCR_TXTDIS;
		if (rc != TWI_SUCCESS) {
			p_pdc->PERIPH_TCR = 0;
			return rc;
		}
		buffer += transfer_size;
		cnt -= transfer_size;
	}

	/* Send remaining byte */
	while (cnt > 0) {
		status = p_twi->TWI_SR;
		if (status & TWI_SR_NACK) {
			return TWI_RECEIVE_NACK;
		}

		if (!(status & TWI_SR_TXRDY)) {
#ifdef FREERTOS_USED
			uint32_t rc = twi_wait_for_interrupt(
					p_twi, TWI_SR_NACK | TWI_SR_TXRDY, 2);
			if (rc != TWI_SUCCESS) {
#	ifdef DEBUG_TWI
			if (rc == TWI_ERROR_TIMEOUT) {
				printf(
						"twi_master_write timed out while waiting for "
						"TXRDY.\r\n");
			} else {
				printf(
						"twi_master_write error occurred while waiting for "
						"TXRDY.\r\n");
			}
#	endif // DEBUG_TWI
				return rc;
			}
#endif // FREERTOS_USED
			continue;
		}
		p_twi->TWI_THR = *buffer++;

		cnt--;
	}

	while (1) {
		status = p_twi->TWI_SR;
		if (status & TWI_SR_NACK) {
			return TWI_RECEIVE_NACK;
		}

		if (status & TWI_SR_TXRDY) {
			break;
		}
#ifdef FREERTOS_USED
		uint32_t rc = twi_wait_for_interrupt(
				p_twi, TWI_SR_NACK | TWI_SR_TXRDY, 2);
		if (rc != TWI_SUCCESS) {
#	ifdef DEBUG_TWI
			if (rc == TWI_ERROR_TIMEOUT) {
				printf(
						"twi_master_write timed out while waiting for "
						"TXRDY.\r\n");
			} else {
				printf(
						"twi_master_write error occurred while waiting for "
						"TXRDY.\r\n");
			}
#	endif // DEBUG_TWI
			return rc;
		}
#endif // FREERTOS_USED
	}

	p_twi->TWI_CR = TWI_CR_STOP;

	while (!(p_twi->TWI_SR & TWI_SR_TXCOMP)) {
		/* Even though it is possible, we do not wait using an interrupt here.
		Experiments have shown that using an interrupt for waiting costs in the
		order of 2-3 % performance, while this loop does typically not take
		long, so that yielding to other tasks does not have any significant
		benefit. */
		if (!timeout--) {
#ifdef DEBUG_TWI
			printf("twi_master_write timed out while waiting for TXCOMP.\r\n");
#endif // DEBUG_TWI
			return TWI_ERROR_TIMEOUT_COMP;
		}
	}

	return TWI_SUCCESS;
}

/**
 * \brief Enable TWI interrupts.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param ul_sources Interrupts to be enabled.
 */
void twi_enable_interrupt(Twi *p_twi, uint32_t ul_sources)
{
	/* Enable the specified interrupts */
	p_twi->TWI_IER = ul_sources;
}

/**
 * \brief Disable TWI interrupts.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param ul_sources Interrupts to be disabled.
 */
void twi_disable_interrupt(Twi *p_twi, uint32_t ul_sources)
{
	/* Disable the specified interrupts */
	p_twi->TWI_IDR = ul_sources;
	/* Dummy read */
	p_twi->TWI_SR;
}

/**
 * \brief Get TWI interrupt status.
 *
 * \param p_twi Pointer to a TWI instance.
 *
 * \retval TWI interrupt status.
 */
uint32_t twi_get_interrupt_status(Twi *p_twi)
{
	return p_twi->TWI_SR;
}

/**
 * \brief Read TWI interrupt mask.
 *
 * \param p_twi Pointer to a TWI instance.
 *
 * \return The interrupt mask value.
 */
uint32_t twi_get_interrupt_mask(Twi *p_twi)
{
	return p_twi->TWI_IMR;
}

/**
 * \brief Reads a byte from the TWI bus.
 *
 * \param p_twi Pointer to a TWI instance.
 *
 * \return The byte read.
 */
uint8_t twi_read_byte(Twi *p_twi)
{
	return p_twi->TWI_RHR;
}

/**
 * \brief Sends a byte of data to one of the TWI slaves on the bus.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param byte The byte to send.
 */
void twi_write_byte(Twi *p_twi, uint8_t uc_byte)
{
	p_twi->TWI_THR = uc_byte;
}

/**
 * \brief Enable TWI slave mode.
 *
 * \param p_twi Pointer to a TWI instance.
 */
void twi_enable_slave_mode(Twi *p_twi)
{
	/* Set Master Disable bit and Slave Disable bit */
	p_twi->TWI_CR = TWI_CR_MSDIS;
	p_twi->TWI_CR = TWI_CR_SVDIS;

	/* Set Slave Enable bit */
	p_twi->TWI_CR = TWI_CR_SVEN;
}

/**
 * \brief Disable TWI slave mode.
 *
 * \param p_twi Pointer to a TWI instance.
 */
void twi_disable_slave_mode(Twi *p_twi)
{
	/* Set Slave Disable bit */
	p_twi->TWI_CR = TWI_CR_SVDIS;
}

/**
 * \brief Initialize TWI slave mode.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param ul_device_addr Device address of the SAM slave device on the I2C bus.
 */
void twi_slave_init(Twi *p_twi, uint32_t ul_device_addr)
{
	/* Disable TWI interrupts */
	p_twi->TWI_IDR = ~0UL;
	p_twi->TWI_SR;

	/* Reset TWI */
	twi_reset(p_twi);

	/* Set slave address in slave mode */
	p_twi->TWI_SMR = TWI_SMR_SADR(ul_device_addr);

	/* Enable slave mode */
	twi_enable_slave_mode(p_twi);
}

/**
 * \brief Set TWI slave address.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param ul_device_addr Device address of the SAM slave device on the I2C bus.
 */
void twi_set_slave_addr(Twi *p_twi, uint32_t ul_device_addr)
{
	/* Set slave address */
	p_twi->TWI_SMR = TWI_SMR_SADR(ul_device_addr);
}

/**
 * \brief Read data from master.
 *
 * \note This function will NOT return until master sends a STOP condition.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param p_data Pointer to the data buffer where data received will be stored.
 *
 * \return Number of bytes read.
 */
uint32_t twi_slave_read(Twi *p_twi, uint8_t *p_data)
{
	uint32_t status, cnt = 0;

	do {
		status = p_twi->TWI_SR;
		if (status & TWI_SR_SVACC) {
			if (!(status & TWI_SR_GACC) &&
				((status & (TWI_SR_SVREAD | TWI_SR_RXRDY))
				 == (TWI_SR_SVREAD | TWI_SR_RXRDY))) {
				*p_data++ = (uint8_t) p_twi->TWI_RHR;
				cnt++;
			}
		} else if ((status & (TWI_SR_EOSACC | TWI_SR_TXCOMP))
					== (TWI_SR_EOSACC | TWI_SR_TXCOMP)) {
			break;
		}
	} while (1);

	return cnt;
}

/**
 * \brief Write data to TWI bus.
 *
 * \note This function will NOT return until master sends a STOP condition.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param p_data Pointer to the data buffer to be sent.
 *
 * \return Number of bytes written.
 */
uint32_t twi_slave_write(Twi *p_twi, uint8_t *p_data)
{
	uint32_t status, cnt = 0;

	do {
		status = p_twi->TWI_SR;
		if (status & TWI_SR_SVACC) {
			if (!(status & (TWI_SR_GACC | TWI_SR_SVREAD)) &&
				(status & TWI_SR_TXRDY)) {
				p_twi->TWI_THR = *p_data++;
				cnt++;
			}
		} else if ((status & (TWI_SR_EOSACC | TWI_SR_TXCOMP))
					== (TWI_SR_EOSACC | TWI_SR_TXCOMP)) {
			break;
		}
	} while (1);

	return cnt;
}

/**
 * \brief Reset TWI.
 *
 * \param p_twi Pointer to a TWI instance.
 */
void twi_reset(Twi *p_twi)
{
	/* Set SWRST bit to reset TWI peripheral */
	p_twi->TWI_CR = TWI_CR_SWRST;
	p_twi->TWI_RHR;
}

/**
 * \brief Get TWI PDC base address.
 *
 * \param p_twi Pointer to a TWI instance.
 *
 * \return TWI PDC registers base for PDC driver to access.
 */
Pdc *twi_get_pdc_base(Twi *p_twi)
{
	Pdc *p_pdc_base = NULL;
#if !SAMG
	if (p_twi == TWI0) {
		p_pdc_base = PDC_TWI0;
	} else
#endif
#ifdef PDC_TWI1
	 if (p_twi == TWI1) {
		p_pdc_base = PDC_TWI1;
	} else
#endif
#ifdef PDC_TWI2
	if (p_twi == TWI2) {
		p_pdc_base = PDC_TWI2;
	} else
#endif
	{
		Assert(false);
	}

	return p_pdc_base;
}

#if (SAM4E || SAM4C || SAMG || SAM4CP || SAM4CM)
/**
 * \brief Enables/Disables write protection mode.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param flag ture for enable, false for disable.
 */
void twi_set_write_protection(Twi *p_twi, bool flag)
{

	p_twi->TWI_WPMR = (flag ? TWI_WPMR_WPEN : 0) | TWI_WP_KEY_VALUE;
}

/**
 * \brief Read the write protection status.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param p_status Pointer to save the status.
 */
void twi_read_write_protection_status(Twi *p_twi, uint32_t *p_status)
{

	*p_status = p_twi->TWI_WPSR;
}
#endif

#if SAMG55
/**
 * \brief Set the prescaler, TLOW:SEXT, TLOW:MEXT and clock high max cycles for SMBUS mode.
 *
 * \param p_twi   Base address of the TWI instance.
 * \param ul_timing Parameter for prescaler, TLOW:SEXT, TLOW:MEXT and clock high max cycles.
 */
void twi_smbus_set_timing(Twi *p_twi, uint32_t ul_timing)
{
	p_twi->TWI_SMBTR = ul_timing;;
}

/**
 * \brief Set length/direction/PEC for alternative command mode.
 *
 * \param p_twi   Base address of the TWI instance.
 * \param ul_alt_cmd Alternative command parameters.
 */
void twi_set_alternative_command(Twi *p_twi, uint32_t ul_alt_cmd)
{
	p_twi->TWI_ACR = ul_alt_cmd;;
}

/**
 * \brief Set the filter for TWI.
 *
 * \param p_twi   Base address of the TWI instance.
 * \param ul_filter   Filter value.
 */
void twi_set_filter(Twi *p_twi, uint32_t ul_filter)
{
	p_twi->TWI_FILTR = ul_filter;;
}

/**
 * \brief A mask can be applied on the slave device address in slave mode in order to allow multiple
 * address answer. For each bit of the MASK field set to one the corresponding SADR bit will be masked.
 *
 * \param p_twi   Base address of the TWI instance.
 * \param ul_mask  Mask value.
 */
void twi_mask_slave_addr(Twi *p_twi, uint32_t ul_mask)
{
	p_twi->TWI_SMR |= TWI_SMR_MASK(ul_mask);
}

/**
 * \brief Set sleepwalking match mode.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param ul_matching_addr1   Address 1 value.
 * \param ul_matching_addr2   Address 2 value.
 * \param ul_matching_addr3   Address 3 value.
 * \param ul_matching_data   Data value.
 * \param flag1 ture for set, false for no.
 * \param flag2 ture for set, false for no.
 * \param flag3 ture for set, false for no.
 * \param flag ture for set, false for no.
 */
void twi_set_sleepwalking(Twi *p_twi,
		uint32_t ul_matching_addr1, bool flag1,
		uint32_t ul_matching_addr2, bool flag2,
		uint32_t ul_matching_addr3, bool flag3,
		uint32_t ul_matching_data, bool flag)
{
	uint32_t temp = 0;

	if (flag1) {
		temp |= TWI_SWMR_SADR1(ul_matching_addr1);
	}

	if (flag2) {
		temp |= TWI_SWMR_SADR2(ul_matching_addr2);
	}

	if (flag3) {
		temp |= TWI_SWMR_SADR3(ul_matching_addr3);
	}

	if (flag) {
		temp |= TWI_SWMR_DATAM(ul_matching_data);
	}

	p_twi->TWI_SWMR = temp;
}
#endif
//@}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
