#include <asf.h>
#include "fixedpointarith.h"

#define NUM_SECTIONS 3
#define NUM_FILTERS 10
#define MAX_AMPLIFICATION ?
#define PEAK_SCALER ?

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
    filters[0].coef[0][0] = float_to_fix(5.62297666e-06);
    ...
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
    filter->peak_reg = (filter->cur_sample > new_reg) ? filter->cur_sample : new_reg;    // note: > works the same for fixed point (test this)
}

int lookup_log(fix_t in) {
    todo if/else for different ranges
}


void TC0_Handler(void) {
    /* Clear status bit to acknowledge interrupt */
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC0, 0);

	// start adc conversion and get each channels values
	adc_start(ADC);
	while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY) {}; // Wait until conversion ready
	uint32_t right_audio_uint = adc_get_channel_value(ADC, 0);
    uint32_t left_audio_uint = adc_get_channel_value(ADC, 0);
    uint32_t volume_wheel_uint = adc_get_channel_value(ADC, 0);
	
    // convert each to fix_t
    fix_t right_audio = uint_to_normalized_fix(right_audio_uint);    // normalized from -1 to 1
    fix_t left_audio = uint_to_normalized_fix(left_audio_uint);
    fix_t volume_wheel = uint_to_positive_fix(volume_wheel_uint);    // normalized from 0 to 1
    
    // mix left and right with (left + right) / 2
    fix_t audio_mixed = fix_div(fix_add(right_audio, left_audio), uint_to_normalized_fix((uint32_t)2));
    
    // scale audio based on MAX_AMPLIFICATION and volume_wheel
    fix_t scaled_audio = fix_mul(audio_mixed, fix_mul(volume_wheel, MAX_AMPLIFICATION));
    
    // run apply filter with each FilterContext on new sample
    for (int i = 0; i < NUM_FILTERS; i++) {
        filters[i].cur_sample = scaled_audio;
        apply_band_pass_filter(&filters[i]);
        apply_peak_detection(&filters[i]);
    }
        
    led_counter += 1;
    if (led_counter > 4410) {    // 4410 counts at 44.1kHz = 100ms
        for (int i = 0; i < NUM_FILTERS; i++) {
            filters[i].log_val = lookup_log(filters[i].peak_reg);
            // output to led
        }
        
        led_counter = 0;
    }
    
    // might need to disable interrupts until done, will miss a few samples if takes too long?
    
    ...
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
    adc_enable_channel(ADC, ADC_CHANNEL_1);
    adc_enable_channel(ADC, ADC_CHANNEL_2);
}

// static void configure_dac(void) {
// 	sysclk_enable_peripheral_clock(ID_DACC);
// 	dacc_reset(DACC);
// 	dacc_disable_trigger(DACC);
// 	dacc_set_transfer_mode(DACC, 0);
// 	dacc_set_channel_selection(DACC, 1);
// 	dacc_enable_channel(DACC,1);
// }

int main(void) {
	sysclk_init();
	board_init();
	
	initialize_coef();
	
	configure_tc();
	configure_adc();
	// configure_dac();
	// configure led stuff
	
	while (true) {}
}
