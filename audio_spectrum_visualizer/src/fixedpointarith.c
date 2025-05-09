/*
 * fixedpointarith.c
 *
 * Created: 3/5/2025 4:26:27 PM
 *  Author: latent
 */ 
#include <stdint.h>
#include "fixedpointarith.h"

//Q8.24

fix_t saturate(int64_t value) {
	if (value > (int32_t)FIXP_MAX) return FIXP_MAX;
	else if (value < (int32_t)FIXP_MIN) return FIXP_MIN;
	else return (fix_t)value;
}

// saturate disabled because too slow
/*
fix_t fix_add(fix_t a, fix_t b) {
	int64_t result = (int64_t)a + b;
	return saturate(result);
}

fix_t fix_sub(fix_t a, fix_t b) {
	int64_t result = (int64_t)a - b;
	return saturate(result);
}

fix_t fix_mul(fix_t a, fix_t b) {
	int64_t result = ((int64_t)a * b) >> 24;
	return saturate(result);
}
*/

fix_t fix_add(fix_t a, fix_t b) {
	return a + b;
}

fix_t fix_sub(fix_t a, fix_t b) {
	return a - b;
}

fix_t fix_mul(fix_t a, fix_t b) {
	return ((int64_t)a * b) >> 24;
}

fix_t fix_div(fix_t a, fix_t b) {
	if (b == 0) { return (a >= 0) ? FIXP_MAX : FIXP_MIN; }    // Division by zero, return maximum value
	int64_t result = ((int64_t)a << 24) / b;
	return result;
}

fix_t int_to_fix(int32_t in) {
	return in << 24;
}

int32_t fix_to_int(fix_t in) {
	return (in >> 24) & 0x000000FF;
}

#ifndef FIXED_POINT_NO_FLOAT
float fix_to_float(fix_t in) {
	return (float)in / (1 << 24);
}

fix_t float_to_fix(float in) {
	if (in > 127.99999994f) in = 127.99999994f;
	else if (in < -128.0f) in = -128.0f;
	return (int32_t)(in * (1 << 24));
}
#endif

fix_t uint_to_normalized_fix(uint32_t in) {
	return ((int32_t)in - 2048) << 13;
}

fix_t uint_to_positive_fix(uint32_t in) {
	return in << 12;
}

uint32_t fix_to_normalized_int(fix_t in) {
	return ((in >> 13) + 2048) & 0xFFF;
}

fix_t fix_abs(fix_t a) {
	return (a < 0) ? -a : a;
}
