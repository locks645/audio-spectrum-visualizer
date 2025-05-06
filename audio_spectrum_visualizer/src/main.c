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
#include "led_interface.h"

#define AUDIO_CHANNEL 13
#define POT_CHANNEL 7

#define NUM_SECTIONS 3
#define NUM_FILTERS 10
#define  MAX_AMPLIFICATION 0x32000000     // 50 in fix_t
#define PEAK_SCALER 0xFFBE77            // 0.999 in fix_t

typedef struct {
	fix_t coef[NUM_SECTIONS][6];  // Coefficients for this filter section (6 coefficients)
	fix_t state[NUM_SECTIONS][4]; // State variables (4 state values)
	fix_t cur_sample;    // current sample
	fix_t peak_reg;
	int log_val;
} FilterContext;

volatile int led_counter = 0;
FilterContext filters[NUM_FILTERS];


// change this to have it do fixed point
// generated externally with calccoefgen.py
void initialize_coef(void) {
	filters[0].coef[0][0] = float_to_fix(1.788007104526562e-07);
	filters[0].coef[0][1] = float_to_fix(3.576014209053124e-07);
	filters[0].coef[0][2] = float_to_fix(1.788007104526562e-07);
	filters[0].coef[0][3] = float_to_fix(1.0);
	filters[0].coef[0][4] = float_to_fix(-1.9886285146404494);
	filters[0].coef[0][5] = float_to_fix(0.9887537426580244);
	filters[0].coef[1][0] = float_to_fix(1.0);
	filters[0].coef[1][1] = float_to_fix(0.0);
	filters[0].coef[1][2] = float_to_fix(-1.0);
	filters[0].coef[1][3] = float_to_fix(1.0);
	filters[0].coef[1][4] = float_to_fix(-1.9917641030212805);
	filters[0].coef[1][5] = float_to_fix(0.9920632588717474);
	filters[0].coef[2][0] = float_to_fix(1.0);
	filters[0].coef[2][1] = float_to_fix(-2.0);
	filters[0].coef[2][2] = float_to_fix(1.0);
	filters[0].coef[2][3] = float_to_fix(1.0);
	filters[0].coef[2][4] = float_to_fix(-1.996611651548705);
	filters[0].coef[2][5] = float_to_fix(0.9966643673347344);
	filters[1].coef[0][0] = float_to_fix(3.689285254565473e-07);
	filters[1].coef[0][1] = float_to_fix(7.378570509130946e-07);
	filters[1].coef[0][2] = float_to_fix(3.689285254565473e-07);
	filters[1].coef[0][3] = float_to_fix(1.0);
	filters[1].coef[0][4] = float_to_fix(-1.9852217320655132);
	filters[1].coef[0][5] = float_to_fix(0.9856898118852827);
	filters[1].coef[1][0] = float_to_fix(1.0);
	filters[1].coef[1][1] = float_to_fix(0.0);
	filters[1].coef[1][2] = float_to_fix(-1.0);
	filters[1].coef[1][3] = float_to_fix(1.0);
	filters[1].coef[1][4] = float_to_fix(-1.9899883328591388);
	filters[1].coef[1][5] = float_to_fix(0.9908217636172243);
	filters[1].coef[2][0] = float_to_fix(1.0);
	filters[1].coef[2][1] = float_to_fix(-2.0);
	filters[1].coef[2][2] = float_to_fix(1.0);
	filters[1].coef[2][3] = float_to_fix(1.0);
	filters[1].coef[2][4] = float_to_fix(-1.9945564857570723);
	filters[1].coef[2][5] = float_to_fix(0.9948212543735838);
	filters[2].coef[0][0] = float_to_fix(7.605903177310472e-07);
	filters[2].coef[0][1] = float_to_fix(1.5211806354620945e-06);
	filters[2].coef[0][2] = float_to_fix(7.605903177310472e-07);
	filters[2].coef[0][3] = float_to_fix(1.0);
	filters[2].coef[0][4] = float_to_fix(-1.980163475363068);
	filters[2].coef[0][5] = float_to_fix(0.9817987083952274);
	filters[2].coef[1][0] = float_to_fix(1.0);
	filters[2].coef[1][1] = float_to_fix(0.0);
	filters[2].coef[1][2] = float_to_fix(-1.0);
	filters[2].coef[1][3] = float_to_fix(1.0);
	filters[2].coef[1][4] = float_to_fix(-1.9866745443690486);
	filters[2].coef[1][5] = float_to_fix(0.9891020380551587);
	filters[2].coef[2][0] = float_to_fix(1.0);
	filters[2].coef[2][1] = float_to_fix(-2.0);
	filters[2].coef[2][2] = float_to_fix(1.0);
	filters[2].coef[2][3] = float_to_fix(1.0);
	filters[2].coef[2][4] = float_to_fix(-1.9915061736849982);
	filters[2].coef[2][5] = float_to_fix(0.9926177399314093);
	filters[3].coef[0][0] = float_to_fix(1.566378730357266e-06);
	filters[3].coef[0][1] = float_to_fix(3.132757460714532e-06);
	filters[3].coef[0][2] = float_to_fix(1.566378730357266e-06);
	filters[3].coef[0][3] = float_to_fix(1.0);
	filters[3].coef[0][4] = float_to_fix(-1.9713161652736904);
	filters[3].coef[0][5] = float_to_fix(0.9768617439154371);
	filters[3].coef[1][0] = float_to_fix(1.0);
	filters[3].coef[1][1] = float_to_fix(0.0);
	filters[3].coef[1][2] = float_to_fix(-1.0);
	filters[3].coef[1][3] = float_to_fix(1.0);
	filters[3].coef[1][4] = float_to_fix(-1.979509434295403);
	filters[3].coef[1][5] = float_to_fix(0.9868134069218718);
	filters[3].coef[2][0] = float_to_fix(1.0);
	filters[3].coef[2][1] = float_to_fix(-2.0);
	filters[3].coef[2][2] = float_to_fix(1.0);
	filters[3].coef[2][3] = float_to_fix(1.0);
	filters[3].coef[2][4] = float_to_fix(-1.9856593151805126);
	filters[3].coef[2][5] = float_to_fix(0.9899185295019383);
	filters[4].coef[0][0] = float_to_fix(3.221486281949527e-06);
	filters[4].coef[0][1] = float_to_fix(6.442972563899054e-06);
	filters[4].coef[0][2] = float_to_fix(3.221486281949527e-06);
	filters[4].coef[0][3] = float_to_fix(1.0);
	filters[4].coef[0][4] = float_to_fix(-1.9520746676089855);
	filters[4].coef[0][5] = float_to_fix(0.9706052095271589);
	filters[4].coef[1][0] = float_to_fix(1.0);
	filters[4].coef[1][1] = float_to_fix(0.0);
	filters[4].coef[1][2] = float_to_fix(-1.0);
	filters[4].coef[1][3] = float_to_fix(1.0);
	filters[4].coef[1][4] = float_to_fix(-1.961316689889374);
	filters[4].coef[1][5] = float_to_fix(0.9838269057802329);
	filters[4].coef[2][0] = float_to_fix(1.0);
	filters[4].coef[2][1] = float_to_fix(-2.0);
	filters[4].coef[2][2] = float_to_fix(1.0);
	filters[4].coef[2][3] = float_to_fix(1.0);
	filters[4].coef[2][4] = float_to_fix(-1.9710888991735034);
	filters[4].coef[2][5] = float_to_fix(0.9865675016582021);
	filters[5].coef[0][0] = float_to_fix(6.61413847590053e-06);
	filters[5].coef[0][1] = float_to_fix(1.322827695180106e-05);
	filters[5].coef[0][2] = float_to_fix(6.61413847590053e-06);
	filters[5].coef[0][3] = float_to_fix(1.0);
	filters[5].coef[0][4] = float_to_fix(-1.9013702273252358);
	filters[5].coef[0][5] = float_to_fix(0.9626882004934494);
	filters[5].coef[1][0] = float_to_fix(1.0);
	filters[5].coef[1][1] = float_to_fix(0.0);
	filters[5].coef[1][2] = float_to_fix(-1.0);
	filters[5].coef[1][3] = float_to_fix(1.0);
	filters[5].coef[1][4] = float_to_fix(-1.909492042210987);
	filters[5].coef[1][5] = float_to_fix(0.9799766583939564);
	filters[5].coef[2][0] = float_to_fix(1.0);
	filters[5].coef[2][1] = float_to_fix(-2.0);
	filters[5].coef[2][2] = float_to_fix(1.0);
	filters[5].coef[2][3] = float_to_fix(1.0);
	filters[5].coef[2][4] = float_to_fix(-1.9280335097284629);
	filters[5].coef[2][5] = float_to_fix(0.9823717948018609);
	filters[6].coef[0][0] = float_to_fix(1.3550394029836814e-05);
	filters[6].coef[0][1] = float_to_fix(2.7100788059673627e-05);
	filters[6].coef[0][2] = float_to_fix(1.3550394029836814e-05);
	filters[6].coef[0][3] = float_to_fix(1.0);
	filters[6].coef[0][4] = float_to_fix(-1.752337728724302);
	filters[6].coef[0][5] = float_to_fix(0.9526887314400103);
	filters[6].coef[1][0] = float_to_fix(1.0);
	filters[6].coef[1][1] = float_to_fix(0.0);
	filters[6].coef[1][2] = float_to_fix(-1.0);
	filters[6].coef[1][3] = float_to_fix(1.0);
	filters[6].coef[1][4] = float_to_fix(-1.7533318087726268);
	filters[6].coef[1][5] = float_to_fix(0.9750639530941086);
	filters[6].coef[2][0] = float_to_fix(1.0);
	filters[6].coef[2][1] = float_to_fix(-2.0);
	filters[6].coef[2][2] = float_to_fix(1.0);
	filters[6].coef[2][3] = float_to_fix(1.0);
	filters[6].coef[2][4] = float_to_fix(-1.7917687921127667);
	filters[6].coef[2][5] = float_to_fix(0.977080353760261);
	filters[7].coef[0][0] = float_to_fix(2.7685187437742187e-05);
	filters[7].coef[0][1] = float_to_fix(5.5370374875484374e-05);
	filters[7].coef[0][2] = float_to_fix(2.7685187437742187e-05);
	filters[7].coef[0][3] = float_to_fix(1.0);
	filters[7].coef[0][4] = float_to_fix(-1.3056210708320786);
	filters[7].coef[0][5] = float_to_fix(0.9400885007623769);
	filters[7].coef[1][0] = float_to_fix(1.0);
	filters[7].coef[1][1] = float_to_fix(0.0);
	filters[7].coef[1][2] = float_to_fix(-1.0);
	filters[7].coef[1][3] = float_to_fix(1.0);
	filters[7].coef[1][4] = float_to_fix(-1.2850978557143788);
	filters[7].coef[1][5] = float_to_fix(0.9688825468551878);
	filters[7].coef[2][0] = float_to_fix(1.0);
	filters[7].coef[2][1] = float_to_fix(-2.0);
	filters[7].coef[2][2] = float_to_fix(1.0);
	filters[7].coef[2][3] = float_to_fix(1.0);
	filters[7].coef[2][4] = float_to_fix(-1.3640123388414866);
	filters[7].coef[2][5] = float_to_fix(0.9703383286124659);
	filters[8].coef[0][0] = float_to_fix(5.63710066693218e-05);
	filters[8].coef[0][1] = float_to_fix(0.0001127420133386436);
	filters[8].coef[0][2] = float_to_fix(5.63710066693218e-05);
	filters[8].coef[0][3] = float_to_fix(1.0);
	filters[8].coef[0][4] = float_to_fix(-0.1044059412679942);
	filters[8].coef[0][5] = float_to_fix(0.9242570992178276);
	filters[8].coef[1][0] = float_to_fix(1.0);
	filters[8].coef[1][1] = float_to_fix(0.0);
	filters[8].coef[1][2] = float_to_fix(-1.0);
	filters[8].coef[1][3] = float_to_fix(1.0);
	filters[8].coef[1][4] = float_to_fix(-0.03955985999969948);
	filters[8].coef[1][5] = float_to_fix(0.9613715281402879);
	filters[8].coef[2][0] = float_to_fix(1.0);
	filters[8].coef[2][1] = float_to_fix(-2.0);
	filters[8].coef[2][2] = float_to_fix(1.0);
	filters[8].coef[2][3] = float_to_fix(1.0);
	filters[8].coef[2][4] = float_to_fix(-0.17304473503430415);
	filters[8].coef[2][5] = float_to_fix(0.9615115636088177);
	filters[9].coef[0][0] = float_to_fix(0.00011428829563131667);
	filters[9].coef[0][1] = float_to_fix(-0.00022857659126263334);
	filters[9].coef[0][2] = float_to_fix(0.00011428829563131667);
	filters[9].coef[0][3] = float_to_fix(1.0);
	filters[9].coef[0][4] = float_to_fix(1.770377580124634);
	filters[9].coef[0][5] = float_to_fix(0.9044370674974141);
	filters[9].coef[1][0] = float_to_fix(1.0);
	filters[9].coef[1][1] = float_to_fix(0.0);
	filters[9].coef[1][2] = float_to_fix(-1.0);
	filters[9].coef[1][3] = float_to_fix(1.0);
	filters[9].coef[1][4] = float_to_fix(1.7741498451983537);
	filters[9].coef[1][5] = float_to_fix(0.9459527266489174);
	filters[9].coef[2][0] = float_to_fix(1.0);
	filters[9].coef[2][1] = float_to_fix(2.0);
	filters[9].coef[2][2] = float_to_fix(1.0);
	filters[9].coef[2][3] = float_to_fix(1.0);
	filters[9].coef[2][4] = float_to_fix(1.8467596592468252);
	filters[9].coef[2][5] = float_to_fix(0.9563539684414235);
}



void apply_band_pass_filter(FilterContext *filter) {
	for (int i = 0; i < NUM_SECTIONS; i++) {
		// Apply biquad filter equation in fixed-point
		// y = coef[i][0] * cur_sample + coef[i][1] * state[i][0] + coef[i][2] * state[i][1] + (0 - coef[i][4] * state[i][2] - coef[i][5] * state[i][3])
		fix_t y = fix_add(
		fix_add(
		fix_add(
		fix_mul(filter->coef[i][0], filter->cur_sample),
		fix_mul(filter->coef[i][1], filter->state[i][0])
		),
		fix_mul(filter->coef[i][2], filter->state[i][1])
		),
		fix_sub(
		fix_sub(
		0,
		fix_mul(filter->coef[i][4], filter->state[i][2])
		),
		fix_mul(filter->coef[i][5], filter->state[i][3])
		)
		);

		// Update state variables
		filter->state[i][1] = filter->state[i][0];  // x[n-2] = x[n-1]
		filter->state[i][0] = filter->cur_sample;       // x[n-1] = current input
		filter->state[i][3] = filter->state[i][2];  // y[n-2] = y[n-1]
		filter->state[i][2] = y;            // y[n-1] = current output

		// Output of the current section becomes input to the next
		filter->cur_sample = y;
	}
	// cur_sample is output
}


void apply_peak_detection(FilterContext *filter) {
	// value from band pass filter is in cur_sample
	fix_t new_reg = fix_mul(filter->peak_reg, PEAK_SCALER);
	fix_t abs_sample = fix_abs(filter->cur_sample);    // absolute value of current sample
	filter->peak_reg = (abs_sample > new_reg) ? abs_sample : new_reg;    // note: > works the same for fixed point
}

// calculated externally with lutgen.c
// fix_t in is intended to be between 0 and 1, if over will output 10, if under will output 0
int lookup_log(fix_t in) {
	if (in > 0xCDDBAB) {
		return 10;
	} else if (in > 0xA5898C) {
		return 9;
	} else if (in > 0x851D30) {
		return 8;
	} else if (in > 0x6B0A98) {
		return 7;
	} else if (in > 0x561355) {
		return 6;
	} else if (in > 0x453757) {
		return 5;
	} else if (in > 0x37A8B5) {
		return 4;
	} else if (in > 0x2CC1DB) {
		return 3;
	} else if (in > 0x23FDA4) {
		return 2;
	} else if (in > 0x1CF0FE) {
		return 1;
	} else {
		return 0;
	}
}


void TC0_Handler(void) {
	/* Clear status bit to acknowledge interrupt */
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC0, 0);
	
	//REG_PIOA_SODR = PIO_PA22;
	
	// start adc conversion and get audio and volume wheel value
	adc_start(ADC);
	while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY) {}; // Wait until conversion ready
	uint32_t audio_uint = adc_get_channel_value(ADC, AUDIO_CHANNEL);
	uint32_t volume_wheel_uint = adc_get_channel_value(ADC, POT_CHANNEL);
	// test here
	
	// convert each to fix_t
	fix_t audio = uint_to_normalized_fix(audio_uint);    // normalized from -1 to 1
	fix_t volume_wheel = uint_to_positive_fix(volume_wheel_uint);    // normalized from 0 to 1
	
	// scale audio based on MAX_AMPLIFICATION and volume_wheel
	fix_t scaled_audio = fix_mul(audio, fix_mul(volume_wheel, MAX_AMPLIFICATION));
	// test here
	
	
	// run apply filter with each FilterContext on new sample
	for (int i = 0; i < NUM_FILTERS; i++) {
		filters[i].cur_sample = scaled_audio;
		apply_band_pass_filter(&filters[i]);
		// test here, output response of filter 5, result is in cur_sample
		
		apply_peak_detection(&filters[i]);
		// test here, result is in peak reg, convert to uint and output to dac
	}
	
// 	uint32_t dac_value = fix_to_normalized_int(filters[2].peak_reg);
// 	while ((dacc_get_interrupt_status(DACC) & DACC_ISR_TXRDY) != DACC_ISR_TXRDY) {};
// 	dacc_write_conversion_data(DACC, dac_value);
	
	led_counter += 1;
	if (led_counter > 2500) {    // 2500 counts at 10khz = 250ms
		tc_stop(TC0, 0);

		//for (volatile int i = 0; i < 3000; ++i);    // busy loop to simulate time it takes for led to output, tests effect of tc disabled on filters
		
		for (int i = 0; i < NUM_FILTERS; i++) {
			filters[i].log_val = lookup_log(filters[i].peak_reg);
			
			// to test: value is 0 to 10, so mul that by 409 and output to dac, should be constant output at voltage corresponding to audio volume
			
			// output to led
			set_volume_column(i, filters[i].log_val);
			//set_volume_column(i, i);
		}
		
		send_colors();
		
		led_counter = 0;
		tc_start(TC0, 0);
	}
	
	//REG_PIOA_CODR = PIO_PA22;
}




static void configure_tc(void) {
	uint32_t ul_sysclk = sysclk_get_cpu_hz(); // Get system clock speed
	uint32_t ul_div;
	uint32_t ul_tcclks;
	
	// Enable the peripheral clock for TC0
	pmc_set_writeprotect(false);
	pmc_enable_periph_clk(ID_TC0);

	// Find the best MCK divisor for 44.1kHz frequency
	tc_find_mck_divisor(10000, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);

	// Initialize TC0, Channel 0 with the selected clock and enable compare match trigger
	tc_init(TC0, 0, ul_tcclks | TC_CMR_CPCTRG);

	// Set the RC value to generate an interrupt every 44.1kHz
	tc_write_rc(TC0, 0, (ul_sysclk / ul_div) / 10000);

	// Enable the interrupt on RC compare match
	NVIC_EnableIRQ((IRQn_Type) ID_TC0); // Enable the TC0 interrupt in NVIC (not needed?)
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS); // Enable interrupt for RC compare match

	// Start the timer
	tc_start(TC0, 0);
}

static void configure_adc(void) {
	pmc_enable_periph_clk(ID_ADC);
	adc_init(ADC, sysclk_get_main_hz(), ADC_FREQ_MAX, ADC_STARTUP_FAST);
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);     // Set timings - standard values
	adc_enable_channel(ADC, AUDIO_CHANNEL);
	adc_enable_channel(ADC, POT_CHANNEL);
}

static void configure_dac(void) {
	pmc_enable_periph_clk(ID_DACC);
	dacc_reset(DACC);
	dacc_disable_trigger(DACC);
	dacc_set_transfer_mode(DACC, 0);
	dacc_set_channel_selection(DACC, 1);
	dacc_enable_channel(DACC,1);
}

int main(void) {
	sysclk_init();
	board_init();
	
	// initialize pin for led interface
	ioport_init();
	ioport_enable_pin(LED_PIN);
	ioport_set_pin_dir(LED_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_PIN, false);
	
	initialize_coef();
	
	configure_tc();
	configure_adc();
	//configure_dac();
	
	while (true) {}
}