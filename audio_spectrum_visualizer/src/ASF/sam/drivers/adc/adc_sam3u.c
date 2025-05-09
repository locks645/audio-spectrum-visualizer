/**
 * \file
 *
 * \brief Analog-to-Digital Converter (ADC/ADC12B) driver for SAM.
 *
 * Copyright (c) 2011-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "adc.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

#if SAM3U

/**
 * \brief Initialize the given ADC with the specified ADC clock and startup time.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_mck Main clock of the device (in Hz).
 * \param ul_adc_clock Analog-to-Digital conversion clock (in Hz).
 * \param ul_startuptime ADC startup time value (value in us). 
 * Please refer to the product datasheet for more details.
 * \param ul_offmode_startuptime ADC off mode startup time value (value in us). 
 * Please refer to the product datasheet for more details.
 *
 * \return 0 on success.
 */
uint32_t adc12b_init(Adc12b *p_adc, const uint32_t ul_mck, const uint32_t ul_adc_clock,
		const uint32_t ul_startuptime, const uint32_t ul_offmode_startuptime)
{
	uint32_t ul_prescal, ul_startup, ul_offmode;
	p_adc->ADC12B_CR = ADC12B_CR_SWRST;

	/* Reset Mode Register */
	p_adc->ADC12B_MR = 0;

	/* Reset PDC transfer */
	p_adc->ADC12B_PTCR = (ADC12B_PTCR_RXTDIS | ADC12B_PTCR_TXTDIS);
	p_adc->ADC12B_RCR = 0;
	p_adc->ADC12B_RNCR = 0;
	ul_prescal = ul_mck / (2 * ul_adc_clock) - 1;
	ul_startup = ((ul_adc_clock / 1000000) * ul_startuptime / 8) - 1;
	ul_offmode = ((ul_adc_clock / 1000000) * ul_offmode_startuptime / 8) -
			1;
	p_adc->ADC12B_MR |=
			ADC12B_MR_PRESCAL(ul_prescal) | ((ul_startup <<
					ADC12B_MR_STARTUP_Pos) &
			ADC12B_MR_STARTUP_Msk);
	p_adc->ADC12B_EMR |= (ul_offmode << 16) & (0xffu << 16);
	return 0;
}

/**
 * \brief Configure conversion resolution.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param resolution ADC resolution.
 */
void adc12b_set_resolution(Adc12b *p_adc, const enum adc_resolution_t resolution)
{
	p_adc->ADC12B_MR |= (resolution << 4) & ADC12B_MR_LOWRES;
}

/**
 * \brief Configure conversion trigger and free run mode.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param trigger Conversion trigger.
 */
void adc12b_configure_trigger(Adc12b *p_adc, const enum adc12b_trigger_t trigger)
{
	p_adc->ADC12B_MR |= trigger;
}

/**
 * \brief Configure ADC power saving mode.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param uc_sleep ADC_MR_SLEEP_NORMAL keeps the ADC Core and reference 
 * voltage circuitry ON between conversions.
 * ADC_MR_SLEEP_SLEEP keeps the ADC Core and reference voltage circuitry
 * OFF between conversions.
 * \param uc_offmode 0 Standby Mode (if Sleep Bit = 1), 1 Off Mode.
 */
void adc12b_configure_power_save(Adc12b *p_adc, const uint8_t uc_sleep,
		uint8_t uc_offmode)
{
	p_adc->ADC12B_MR |= ((uc_sleep << 5) & ADC12B_MR_SLEEP);
	p_adc->ADC12B_EMR |= uc_offmode;
}

/**
 * \brief Configure ADC timing.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_sh ADC sample and hold time = uc_sh / ADC clock.
 */
void adc12b_configure_timing(Adc12b *p_adc, const uint32_t ul_sh)
{
	p_adc->ADC12B_MR |= ADC12B_MR_SHTIM(ul_sh);
}

/**
 * \brief Start ADC conversion.
 *
 * \note If one of the hardware event is selected as ADC trigger, 
 * this function can NOT start ADC conversion.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc12b_start(Adc12b *p_adc)
{
	p_adc->ADC12B_CR = ADC12B_CR_START;
}

/**
 * \brief Reset ADC.
 * \param p_adc Pointer to an ADC instance.
 */
void adc12b_reset(Adc12b *p_adc)
{
	p_adc->ADC12B_CR = ADC12B_CR_SWRST;
}

/**
 * \brief Enable the specified ADC channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param adc_ch ADC channel number.
 */
void adc12b_enable_channel(Adc12b *p_adc, const enum adc_channel_num_t adc_ch)
{
	p_adc->ADC12B_CHER = 1 << adc_ch;
}

/**
 * \brief Enable all ADC channels.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc12b_enable_all_channel(Adc12b *p_adc)
{
	p_adc->ADC12B_CHER = 0xFF;
}

/**
 * \brief Disable the specified ADC channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param adc_ch ADC channel number.
 */
void adc12b_disable_channel(Adc12b *p_adc, const enum adc_channel_num_t adc_ch)
{
	p_adc->ADC12B_CHDR = 1 << adc_ch;
}

/**
 * \brief Disable all ADC channel.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc12b_disable_all_channel(Adc12b *p_adc)
{
	p_adc->ADC12B_CHDR = 0xFF;
}

/**
 * \brief Read the ADC channel status.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param adc_ch ADC channel number.
 *
 * \retval 1 if channel is enabled.
 * \retval 0 if channel is disabled.
 */
uint32_t adc12b_get_channel_status(const Adc12b *p_adc, const enum adc_channel_num_t adc_ch)
{
	return p_adc->ADC12B_CHSR & (1 << adc_ch);
}

/**
 * \brief Read the ADC result data of the specified channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param adc_ch ADC channel number.
 *
 * \return ADC value of the specified channel.
 */
uint32_t adc12b_get_channel_value(const Adc12b *p_adc,const enum adc_channel_num_t adc_ch)
{
	uint32_t dwData = 0;

	if (15 >= adc_ch) {
		dwData = *(p_adc->ADC12B_CDR + adc_ch);
	}

	return dwData;
}

/**
 * \brief Read the last ADC result data.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return ADC latest value.
 */
uint32_t adc12b_get_latest_value(const Adc12b *p_adc)
{
	return p_adc->ADC12B_LCDR;
}

/**
 * \brief Enable differential input for all channels.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc12b_enable_differential_input(Adc12b *p_adc)
{
	p_adc->ADC12B_ACR |= (0x01u << 16);
}

/**
 * \brief Disable differential input for the specified channel.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc12b_disable_differential_input(Adc12b *p_adc)
{
	p_adc->ADC12B_ACR &= (0x01u << 16);
}

/**
 * \brief Enable analog signal offset for the specified channel.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc12b_enable_input_offset(Adc12b *p_adc)
{
	p_adc->ADC12B_ACR |= (0x01u << 17);
}

/**
 * \brief Disable analog signal offset for the specified channel.
 *
 * \param p_adc Pointer to an ADC instance.
 */
void adc12b_disable_input_offset(Adc12b *p_adc)
{
	p_adc->ADC12B_ACR &= (0x01u << 17);
}

/**
 * \brief Configure input gain for the specified channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param gain Gain value for the input.
 */
void adc12b_set_input_gain(Adc12b *p_adc, const enum adc_gainvalue_t gain)
{
	p_adc->ADC12B_ACR |= (0x03u & gain);
}

/**
 * \brief Return the actual ADC clock.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_mck Main clock of the device (in Hz).
 *
 * \retval 0 The actual ADC clock (in Hz).
 */
uint32_t adc12b_get_actual_adc_clock(const Adc12b *p_adc, const uint32_t ul_mck)
{
	uint32_t ul_adcfreq;
	uint32_t ul_prescal;

	/* ADCClock = MCK / ( (PRESCAL+1) * 2 ) */
	ul_prescal = ((p_adc->ADC12B_MR & ADC12B_MR_PRESCAL_Msk) >>
			ADC12B_MR_PRESCAL_Pos);
	ul_adcfreq = ul_mck / ((ul_prescal + 1) * 2);
	return ul_adcfreq;
}

/**
 * \brief Enable ADC interrupts.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_source Interrupts to be enabled.
 */
void adc12b_enable_interrupt(Adc12b *p_adc, const uint32_t ul_source)
{
	p_adc->ADC12B_IER = ul_source;
}

/**
 * \brief Disable ADC interrupts.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param ul_source Interrupts to be disabled.
 */
void adc12b_disable_interrupt(Adc12b *p_adc, const uint32_t ul_source)
{
	p_adc->ADC12B_IDR = ul_source;
}

/** \brief Read ADC interrupt mask.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return The interrupt mask value.
 */
uint32_t adc12b_get_interrupt_mask(const Adc12b *p_adc)
{
	return p_adc->ADC12B_IMR;
}

/**
 * \brief Read ADC interrupt status.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \retval ADC interrupt status.
 */
uint32_t adc12b_get_status(const Adc12b *p_adc)
{
	return p_adc->ADC12B_SR;
}

/**
 * \brief Adapt performance versus power consumption.
 *
 * \note Please refer to ADC Characteristics in the product datasheet 
 * for more details.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param uc_ibctl ADC Bias current control.
 */
void adc12b_set_bias_current(Adc12b *p_adc, const uint8_t uc_ibctl)
{
	p_adc->ADC12B_ACR |= ADC12B_ACR_IBCTL(uc_ibctl);
}

/**
 * \brief Get PDC registers base address.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return ADC PDC register base address.
 */
Pdc *adc12b_get_pdc_base(const Adc12b *p_adc)
{
	UNUSED(p_adc);
	return PDC_ADC12B;
}
#endif

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
