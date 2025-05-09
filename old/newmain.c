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
#define NUM_FILTERS 10
#define MAX_AMPLIFICATION 0xa000000      // 10 in fix_t
#define PEAK_SCALER 0xf33333            // 0.95 in fix_t

typedef struct {
	fix_t coef[NUM_SECTIONS][6];  // Coefficients for this filter section (6 coefficients)
	fix_t state[NUM_SECTIONS][4]; // State variables (4 state values)
	fix_t cur_sample;    // current sample
	fix_t peak_reg;
	int log_val;
} FilterContext;

int led_counter = 0;
FilterContext filters[NUM_FILTERS];

void initialize_coef(void) {
	filters[0].coef[0][0] = float_to_fix(2.891346178661002e-12);
	filters[0].coef[0][1] = float_to_fix(5.782692357322004e-12);
	filters[0].coef[0][2] = float_to_fix(2.891346178661002e-12);
	filters[0].coef[0][3] = float_to_fix(1.0);
	filters[0].coef[0][4] = float_to_fix(-1.9997069905870444);
	filters[0].coef[0][5] = float_to_fix(0.9997150888764177);
	filters[0].coef[1][0] = float_to_fix(1.0);
	filters[0].coef[1][1] = float_to_fix(0.0);
	filters[0].coef[1][2] = float_to_fix(-1.0);
	filters[0].coef[1][3] = float_to_fix(1.0);
	filters[0].coef[1][4] = float_to_fix(-1.999842529521508);
	filters[0].coef[1][5] = float_to_fix(0.9998513619666114);
	filters[0].coef[2][0] = float_to_fix(1.0);
	filters[0].coef[2][1] = float_to_fix(-2.0);
	filters[0].coef[2][2] = float_to_fix(1.0);
	filters[0].coef[2][3] = float_to_fix(1.0);
	filters[0].coef[2][4] = float_to_fix(-1.999856280442551);
	filters[0].coef[2][5] = float_to_fix(0.9998637066572141);
	filters[1].coef[0][0] = float_to_fix(2.910577843561319e-11);
	filters[1].coef[0][1] = float_to_fix(5.821155687122638e-11);
	filters[1].coef[0][2] = float_to_fix(2.910577843561319e-11);
	filters[1].coef[0][3] = float_to_fix(1.0);
	filters[1].coef[0][4] = float_to_fix(-1.9993470960985924);
	filters[1].coef[0][5] = float_to_fix(0.999384853510787);
	filters[1].coef[1][0] = float_to_fix(1.0);
	filters[1].coef[1][1] = float_to_fix(0.0);
	filters[1].coef[1][2] = float_to_fix(-1.0);
	filters[1].coef[1][3] = float_to_fix(1.0);
	filters[1].coef[1][4] = float_to_fix(-1.9996378695128199);
	filters[1].coef[1][5] = float_to_fix(0.9996790530909672);
	filters[1].coef[2][0] = float_to_fix(1.0);
	filters[1].coef[2][1] = float_to_fix(-2.0);
	filters[1].coef[2][2] = float_to_fix(1.0);
	filters[1].coef[2][3] = float_to_fix(1.0);
	filters[1].coef[2][4] = float_to_fix(-1.9996710791006034);
	filters[1].coef[2][5] = float_to_fix(0.9997057060252901);
	filters[2].coef[0][0] = float_to_fix(2.9288160977780906e-10);
	filters[2].coef[0][1] = float_to_fix(5.857632195556181e-10);
	filters[2].coef[0][2] = float_to_fix(2.9288160977780906e-10);
	filters[2].coef[0][3] = float_to_fix(1.0);
	filters[2].coef[0][4] = float_to_fix(-1.9984960976725596);
	filters[2].coef[0][5] = float_to_fix(0.9986721023200189);
	filters[2].coef[1][0] = float_to_fix(1.0);
	filters[2].coef[1][1] = float_to_fix(0.0);
	filters[2].coef[1][2] = float_to_fix(-1.0);
	filters[2].coef[1][3] = float_to_fix(1.0);
	filters[2].coef[1][4] = float_to_fix(-1.999115057109782);
	filters[2].coef[1][5] = float_to_fix(0.9993070652104443);
	filters[2].coef[2][0] = float_to_fix(1.0);
	filters[2].coef[2][1] = float_to_fix(-2.0);
	filters[2].coef[2][2] = float_to_fix(1.0);
	filters[2].coef[2][3] = float_to_fix(1.0);
	filters[2].coef[2][4] = float_to_fix(-1.9992031552376934);
	filters[2].coef[2][5] = float_to_fix(0.9993645974027724);
	filters[3].coef[0][0] = float_to_fix(2.9447355009274148e-09);
	filters[3].coef[0][1] = float_to_fix(5.8894710018548295e-09);
	filters[3].coef[0][2] = float_to_fix(2.9447355009274148e-09);
	filters[3].coef[0][3] = float_to_fix(1.0);
	filters[3].coef[0][4] = float_to_fix(-1.9963146251884853);
	filters[3].coef[0][5] = float_to_fix(0.9971346906923453);
	filters[3].coef[1][0] = float_to_fix(1.0);
	filters[3].coef[1][1] = float_to_fix(0.0);
	filters[3].coef[1][2] = float_to_fix(-1.0);
	filters[3].coef[1][3] = float_to_fix(1.0);
	filters[3].coef[1][4] = float_to_fix(-1.997609309561098);
	filters[3].coef[1][5] = float_to_fix(0.998504265074134);
	filters[3].coef[2][0] = float_to_fix(1.0);
	filters[3].coef[2][1] = float_to_fix(-2.0);
	filters[3].coef[2][2] = float_to_fix(1.0);
	filters[3].coef[2][3] = float_to_fix(1.0);
	filters[3].coef[2][4] = float_to_fix(-1.9978758610708518);
	filters[3].coef[2][5] = float_to_fix(0.9986283799276342);
	filters[4].coef[0][0] = float_to_fix(2.9554749354705246e-08);
	filters[4].coef[0][1] = float_to_fix(5.910949870941049e-08);
	filters[4].coef[0][2] = float_to_fix(2.9554749354705246e-08);
	filters[4].coef[0][3] = float_to_fix(1.0);
	filters[4].coef[0][4] = float_to_fix(-1.9900059659227725);
	filters[4].coef[0][5] = float_to_fix(0.9938227858037596);
	filters[4].coef[1][0] = float_to_fix(1.0);
	filters[4].coef[1][1] = float_to_fix(0.0);
	filters[4].coef[1][2] = float_to_fix(-1.0);
	filters[4].coef[1][3] = float_to_fix(1.0);
	filters[4].coef[1][4] = float_to_fix(-1.992604413049482);
	filters[4].coef[1][5] = float_to_fix(0.9967729948325093);
	filters[4].coef[2][0] = float_to_fix(1.0);
	filters[4].coef[2][1] = float_to_fix(-2.0);
	filters[4].coef[2][2] = float_to_fix(1.0);
	filters[4].coef[2][3] = float_to_fix(1.0);
	filters[4].coef[2][4] = float_to_fix(-1.9935347642515249);
	filters[4].coef[2][5] = float_to_fix(0.9970402991106848);
	filters[5].coef[0][0] = float_to_fix(2.954915068939763e-07);
	filters[5].coef[0][1] = float_to_fix(5.909830137879527e-07);
	filters[5].coef[0][2] = float_to_fix(2.954915068939763e-07);
	filters[5].coef[0][3] = float_to_fix(1.0);
	filters[5].coef[0][4] = float_to_fix(-1.9689936592184152);
	filters[5].coef[0][5] = float_to_fix(0.9867081287989942);
	filters[5].coef[1][0] = float_to_fix(1.0);
	filters[5].coef[1][1] = float_to_fix(0.0);
	filters[5].coef[1][2] = float_to_fix(-1.0);
	filters[5].coef[1][3] = float_to_fix(1.0);
	filters[5].coef[1][4] = float_to_fix(-1.9736680644655535);
	filters[5].coef[1][5] = float_to_fix(0.9930459415446369);
	filters[5].coef[2][0] = float_to_fix(1.0);
	filters[5].coef[2][1] = float_to_fix(-2.0);
	filters[5].coef[2][2] = float_to_fix(1.0);
	filters[5].coef[2][3] = float_to_fix(1.0);
	filters[5].coef[2][4] = float_to_fix(-1.977316931835489);
	filters[5].coef[2][5] = float_to_fix(0.9936184002194585);
	filters[6].coef[0][0] = float_to_fix(2.9302236088285575e-06);
	filters[6].coef[0][1] = float_to_fix(5.860447217657115e-06);
	filters[6].coef[0][2] = float_to_fix(2.9302236088285575e-06);
	filters[6].coef[0][3] = float_to_fix(1.0);
	filters[6].coef[0][4] = float_to_fix(-1.889982629757749);
	filters[6].coef[0][5] = float_to_fix(0.9715149667369086);
	filters[6].coef[1][0] = float_to_fix(1.0);
	filters[6].coef[1][1] = float_to_fix(0.0);
	filters[6].coef[1][2] = float_to_fix(-1.0);
	filters[6].coef[1][3] = float_to_fix(1.0);
	filters[6].coef[1][4] = float_to_fix(-1.8955904009142626);
	filters[6].coef[1][5] = float_to_fix(0.985057980193634);
	filters[6].coef[2][0] = float_to_fix(1.0);
	filters[6].coef[2][1] = float_to_fix(-2.0);
	filters[6].coef[2][2] = float_to_fix(1.0);
	filters[6].coef[2][3] = float_to_fix(1.0);
	filters[6].coef[2][4] = float_to_fix(-1.9108985887558692);
	filters[6].coef[2][5] = float_to_fix(0.9862575063786126);
	filters[7].coef[0][0] = float_to_fix(2.8556305138276706e-05);
	filters[7].coef[0][1] = float_to_fix(5.711261027655341e-05);
	filters[7].coef[0][2] = float_to_fix(2.8556305138276706e-05);
	filters[7].coef[0][3] = float_to_fix(1.0);
	filters[7].coef[0][4] = float_to_fix(-1.574785526601852);
	filters[7].coef[0][5] = float_to_fix(0.9394726191490261);
	filters[7].coef[1][0] = float_to_fix(1.0);
	filters[7].coef[1][1] = float_to_fix(0.0);
	filters[7].coef[1][2] = float_to_fix(-1.0);
	filters[7].coef[1][3] = float_to_fix(1.0);
	filters[7].coef[1][4] = float_to_fix(-1.5658389055683184);
	filters[7].coef[1][5] = float_to_fix(0.9681581724416568);
	filters[7].coef[2][0] = float_to_fix(1.0);
	filters[7].coef[2][1] = float_to_fix(-2.0);
	filters[7].coef[2][2] = float_to_fix(1.0);
	filters[7].coef[2][3] = float_to_fix(1.0);
	filters[7].coef[2][4] = float_to_fix(-1.629860058321934);
	filters[7].coef[2][5] = float_to_fix(0.9704299963608687);
	filters[8].coef[0][0] = float_to_fix(0.0002684208225861774);
	filters[8].coef[0][1] = float_to_fix(0.0005368416451723548);
	filters[8].coef[0][2] = float_to_fix(0.0002684208225861774);
	filters[8].coef[0][3] = float_to_fix(1.0);
	filters[8].coef[0][4] = float_to_fix(-0.4160138992583039);
	filters[8].coef[0][5] = float_to_fix(0.8735848107962493);
	filters[8].coef[1][0] = float_to_fix(1.0);
	filters[8].coef[1][1] = float_to_fix(0.0);
	filters[8].coef[1][2] = float_to_fix(-1.0);
	filters[8].coef[1][3] = float_to_fix(1.0);
	filters[8].coef[1][4] = float_to_fix(-0.3179079498909474);
	filters[8].coef[1][5] = float_to_fix(0.9341085974384078);
	filters[8].coef[2][0] = float_to_fix(1.0);
	filters[8].coef[2][1] = float_to_fix(-2.0);
	filters[8].coef[2][2] = float_to_fix(1.0);
	filters[8].coef[2][3] = float_to_fix(1.0);
	filters[8].coef[2][4] = float_to_fix(-0.5385367523853137);
	filters[8].coef[2][5] = float_to_fix(0.9357816155835763);
	filters[9].coef[0][0] = float_to_fix(0.002348805297391535);
	filters[9].coef[0][1] = float_to_fix(-0.00469761059478307);
	filters[9].coef[0][2] = float_to_fix(0.002348805297391535);
	filters[9].coef[0][3] = float_to_fix(1.0);
	filters[9].coef[0][4] = float_to_fix(1.7158221709891857);
	filters[9].coef[0][5] = float_to_fix(0.7444136492186384);
	filters[9].coef[1][0] = float_to_fix(1.0);
	filters[9].coef[1][1] = float_to_fix(0.0);
	filters[9].coef[1][2] = float_to_fix(-1.0);
	filters[9].coef[1][3] = float_to_fix(1.0);
	filters[9].coef[1][4] = float_to_fix(1.681111521926648);
	filters[9].coef[1][5] = float_to_fix(0.7949671328259066);
	filters[9].coef[2][0] = float_to_fix(1.0);
	filters[9].coef[2][1] = float_to_fix(2.0);
	filters[9].coef[2][2] = float_to_fix(1.0);
	filters[9].coef[2][3] = float_to_fix(1.0);
	filters[9].coef[2][4] = float_to_fix(1.9342366866324607);
	filters[9].coef[2][5] = float_to_fix(0.9423172718481085);
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
	filter->peak_reg = (abs_sample > new_reg) ? abs_sample : new_reg;    // note: > works the same for fixed point (test this)
}

int lookup_log(fix_t in) {
	if (in > 0x1000000) {    // log(MAX_AMPLIFICATION) = log(10) = 1
		return 9;
	} else if (in > 0xf4493d) {    // log(9) = 0.9542425094
		return 8;
	} else if (in > 0xe730e8) {    // 8 0.9030899870
		return 7; 
	} else if (in > 0xd85858) {    // 7 0.8450980400
		return 6;
	} else if (in > 0xc734ec) {    // 6 0.7781512504
		return 5;
	} else if (in > 0xb2efb3) {    // 5 0.6989700043
		return 4;
	} else if (in > 0x9a209b) {    // 4 0.6020599913
		return 3;
	} else if (in > 0x7a249e) {    // 3 0.4771212547
		return 2;
	} else if (in > 0x4d104d) {    // 2 0.3010299957
		return 1;
	} else if (in > 0) {    //  1 0
		return 0;
	}
	else {
		return 0;
	}
	
}


void TC0_Handler(void) {
	/* Clear status bit to acknowledge interrupt */
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC0, 0);

	// start adc conversion and get audio and volume wheel value
	adc_start(ADC);
	while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY) {}; // Wait until conversion ready
	uint32_t audio_uint = adc_get_channel_value(ADC, 0);
	uint32_t volume_wheel_uint = adc_get_channel_value(ADC, 1);
	
	// convert each to fix_t
	fix_t audio = uint_to_normalized_fix(audio_uint);    // normalized from -1 to 1
	fix_t volume_wheel = uint_to_positive_fix(volume_wheel_uint);    // normalized from 0 to 1
	
	// scale audio based on MAX_AMPLIFICATION and volume_wheel
	fix_t scaled_audio = fix_mul(audio, fix_mul(volume_wheel, MAX_AMPLIFICATION));
	// convert this back to uint and output to dac
	uint32_t dac_value = fix_to_normalized_int(scaled_audio);
	while ((dacc_get_interrupt_status(DACC) & DACC_ISR_TXRDY) != DACC_ISR_TXRDY) {};
	dacc_write_conversion_data(DACC, dac_value);
	
	// run apply filter with each FilterContext on new sample
	for (int i = 0; i < NUM_FILTERS; i++) {
		filters[i].cur_sample = scaled_audio;
		apply_band_pass_filter(&filters[i]);
		// result is in cur_sample, convert to uint and output to dac
		apply_peak_detection(&filters[i]);
		// result is in peak reg, convert to uint and output to dac
	}
	
	led_counter += 1;
	if (led_counter > 4410) {    // 4410 counts at 44.1kHz = 100ms
		tc_stop(TC0, 0);

		for (int i = 0; i < NUM_FILTERS; i++) {
			filters[i].log_val = lookup_log(filters[i].peak_reg);
			// to test: value is 0 to 9, so mul that by 455 and output to dac, should be constant output at voltage corresponding to audio volume
			// output to led
		}
		
		led_counter = 0;
		tc_start(TC0, 0);
	}
	
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
	NVIC_EnableIRQ((IRQn_Type) ID_TC0); // Enable the TC0 interrupt in NVIC (not needed?)
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS); // Enable interrupt for RC compare match

	// Start the timer
	tc_start(TC0, 0);
}


static void configure_adc(void) {
	pmc_enable_periph_clk(ID_ADC);
	adc_init(ADC, sysclk_get_main_hz(), ADC_FREQ_MAX, ADC_STARTUP_FAST);
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);     // Set timings - standard values
	adc_enable_channel(ADC, ADC_CHANNEL_0);
	adc_enable_channel(ADC, ADC_CHANNEL_1);
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
	
	initialize_coef();
	
	configure_tc();
	configure_adc();
	configure_dac();
	// configure led stuff
	
	while (true) {}
}
