#include <stdio.h>
#include "fixedpointarith.c"

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

int main() {
    printf("%X\n", float_to_fix(50.0));
    
    // Loop through the float range
    for (float f = 0.0; f <= 1.0; f += 0.001) {
        fix_t fixed_input = float_to_fix(f);  // Convert float to fixed-point
        int result = lookup_log(fixed_input); // Call the lookup_log function with the fixed-point input
        printf("Input: %f -> Fixed-point: %X -> Lookup Log: %d\n", f, fixed_input, result);
    }

    return 0;
} 
