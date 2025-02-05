/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include "fixedpointarith.h"

#define NUM_SECTIONS 3



// Coefficients in fixed-point format (Q8.24)
fix_t coef[NUM_SECTIONS][6];

void initialize_coef(void) {
	coef[0][0] = float_to_fix(2.81148833e-06);
	coef[0][1] = float_to_fix(5.62297666e-06);
	coef[0][2] = float_to_fix(2.81148833e-06);
	coef[0][3] = float_to_fix(1.00000000e+00);
	coef[0][4] = float_to_fix(-1.95212101e+00);
	coef[0][5] = float_to_fix(9.71903238e-01);
	coef[1][0] = float_to_fix(1.00000000e+00);
	coef[1][1] = float_to_fix(0.00000000e+00);
	coef[1][2] = float_to_fix(-1.00000000e+00);
	coef[1][3] = float_to_fix(1.00000000e+00);
	coef[1][4] = float_to_fix(-1.96095667e+00);
	coef[1][5] = float_to_fix(9.84643863e-01);
	coef[2][0] = float_to_fix(1.00000000e+00);
	coef[2][1] = float_to_fix(-2.00000000e+00);
	coef[2][2] = float_to_fix(1.00000000e+00);
	coef[2][3] = float_to_fix(1.00000000e+00);
	coef[2][4] = float_to_fix(-1.97031331e+00);
	coef[2][5] = float_to_fix(9.87066388e-01);
}



// State variables in fixed-point format [x[n-1], x[n-2], y[n-1], y[n-2]]
fix_t state[NUM_SECTIONS][4] = {0};

void TC0_Handler(void) {
	/* Clear status bit to acknowledge interrupt */
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC0, 0);

	// Start ADC conversion
	adc_start(ADC);
	while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY) {}; // Wait until conversion ready
	uint32_t result = adc_get_latest_value(ADC);

	// Convert ADC result to normalized fixed-point (-1.0 to 1.0)
	fix_t output = uint_to_normalized_fix(result);

	for (int i = 0; i < NUM_SECTIONS; i++) {
		// Apply biquad filter equation in fixed-point
		fix_t y = fix_add(
		fix_add(
		fix_add(
		fix_mul(coef[i][0], output),
		fix_mul(coef[i][1], state[i][0])
		),
		fix_mul(coef[i][2], state[i][1])
		),
		fix_sub(
		fix_sub(
		0,
		fix_mul(coef[i][4], state[i][2])
		),
		fix_mul(coef[i][5], state[i][3])
		)
		);

		// Update state variables
		state[i][1] = state[i][0];  // x[n-2] = x[n-1]
		state[i][0] = output;       // x[n-1] = current input
		state[i][3] = state[i][2];  // y[n-2] = y[n-1]
		state[i][2] = y;            // y[n-1] = current output

		// Output of the current section becomes input to the next
		output = y;
	}

	// Convert fixed-point output back to DAC value (0-4095)
	uint32_t dac_value = fix_to_normalized_int(output);

	// Wait for DAC to be ready and write the value
	while ((dacc_get_interrupt_status(DACC) & DACC_ISR_TXRDY) != DACC_ISR_TXRDY) {};
	dacc_write_conversion_data(DACC, dac_value);
}







static void configure_tc(void) {
	uint32_t ul_sysclk = sysclk_get_cpu_hz(); // Get system clock speed
	uint32_t ul_div;
	uint32_t ul_tcclks;
	
	// Enable the peripheral clock for TC0
	pmc_set_writeprotect(false);
	pmc_enable_periph_clk(ID_TC0);

	// Find the best MCK divisor for 44.1kHz frequency
	tc_find_mck_divisor(44100, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);

	// Initialize TC0, Channel 0 with the selected clock and enable compare match trigger
	tc_init(TC0, 0, ul_tcclks | TC_CMR_CPCTRG);

	// Set the RC value to generate an interrupt every 44.1kHz
	tc_write_rc(TC0, 0, (ul_sysclk / ul_div) / 44100);

	// Enable the interrupt on RC compare match
	NVIC_EnableIRQ((IRQn_Type) ID_TC0); // Enable the TC0 interrupt in NVIC
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS); // Enable interrupt for RC compare match

	// Start the timer
	tc_start(TC0, 0);
}


static void configure_adc(void) {
	pmc_enable_periph_clk(ID_ADC);
	adc_init(ADC, sysclk_get_main_hz(), ADC_FREQ_MAX, ADC_STARTUP_FAST);
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);     // Set timings - standard values
	adc_enable_channel(ADC, ADC_CHANNEL_0);
}

static void configure_dac(void) {
	sysclk_enable_peripheral_clock(ID_DACC);
	dacc_reset(DACC);
	dacc_disable_trigger(DACC);
	dacc_set_transfer_mode(DACC, 0);
	dacc_set_channel_selection(DACC, 1);
	dacc_enable_channel(DACC,1);
}

int main(void) {
	sysclk_init();
	board_init();
	
	initialize_coef();
	
	configure_tc();
	configure_adc();
	configure_dac();
	
	
	while (true) {}
}
