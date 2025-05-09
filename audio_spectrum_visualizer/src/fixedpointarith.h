/*
 * fixedpointarith.h
 *
 * Created: 3/5/2025 4:26:59 PM
 *  Author: latent
 */ 

#ifndef FIXEDPOINTARITH_H_
#define FIXEDPOINTARITH_H_

#include <stdint.h>

// Define fixed-point type (Q8.24 format)
typedef int32_t fix_t;

// Fixed-point limits (Q8.24 format)
#define FIXP_MAX 0x7FFFFFFF
#define FIXP_MIN 0x80000000

/**
 * @brief Saturates a 64-bit value to the Q8.24 fixed-point range.
 * @param value A 64-bit integer value.
 * @return Saturated Q8.24 fixed-point value.
 */
fix_t saturate(int64_t value);

/**
 * @brief Adds two Q8.24 fixed-point numbers.
 * @param a First operand.
 * @param b Second operand.
 * @return Sum of a and b as a Q8.24 fixed-point number.
 */
fix_t fix_add(fix_t a, fix_t b);

/**
 * @brief Subtracts two Q8.24 fixed-point numbers.
 * @param a First operand.
 * @param b Second operand.
 * @return Difference of a and b as a Q8.24 fixed-point number.
 */
fix_t fix_sub(fix_t a, fix_t b);

/**
 * @brief Multiplies two Q8.24 fixed-point numbers.
 * @param a First operand.
 * @param b Second operand.
 * @return Product of a and b as a Q8.24 fixed-point number.
 */
fix_t fix_mul(fix_t a, fix_t b);

/**
 * @brief Divides two Q8.24 fixed-point numbers.
 * @param a Dividend.
 * @param b Divisor.
 * @return Quotient of a and b as a Q8.24 fixed-point number.
 */
fix_t fix_div(fix_t a, fix_t b);

/**
 * @brief Converts an integer to a Q8.24 fixed-point number.
 * @param in Integer value.
 * @return Q8.24 fixed-point representation of the input.
 */
fix_t int_to_fix(int32_t in);

/**
 * @brief Converts a Q8.24 fixed-point number to an integer.
 * @param in Q8.24 fixed-point value.
 * @return Integer representation of the input.
 */
int32_t fix_to_int(fix_t in);

#ifndef FIXED_POINT_NO_FLOAT
/**
 * @brief Converts a floating-point number to a Q8.24 fixed-point number.
 * @param in Floating-point value.
 * @return Q8.24 fixed-point representation of the input.
 */
fix_t float_to_fix(float in);

/**
 * @brief Converts a Q8.24 fixed-point number to a floating-point number.
 * @param in Q8.24 fixed-point value.
 * @return Floating-point representation of the input.
 */
float fix_to_float(fix_t in);
#endif

/**
 * @brief Converts a 12-bit unsigned integer to a normalized Q8.24 fixed-point number (-1.0 to 1.0).
 * @param in 12-bit unsigned integer (0 to 4095).
 * @return Q8.24 fixed-point normalized value.
 */
fix_t uint_to_normalized_fix(uint32_t in);

// same as above but 0 to 1
fix_t uint_to_positive_fix(uint32_t in);

/**
 * @brief Converts a normalized Q8.24 fixed-point number (-1.0 to 1.0) to a 12-bit unsigned integer.
 * @param in Q8.24 fixed-point normalized value.
 * @return 12-bit unsigned integer (0 to 4095).
 */
uint32_t fix_to_normalized_int(fix_t in);

// fixed point absolute value
fix_t fix_abs(fix_t a);


#endif /* FIXEDPOINTARITH_H_ */