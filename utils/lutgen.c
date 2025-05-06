// gcc lutgen.c -lm -o lutgen

// y = log(x * 11, 11) * 11
// y / 11 = log(x * 11, 11)
// 11 ^ (y / 11) = x * 11
// (11 ^ (y / 11)) / 11 = x

#include <stdio.h>
#include <math.h>
#include "fixedpointarith.c"

double calcinvlog(double y) {
    return pow(11, (y / 11)) / 11;
}

int main() {
    for (int i = 0; i <= 10; i++) {
        double x = calcinvlog((double)i);
        fix_t x_fix = float_to_fix(x);
        printf("%f : %d ... 0x%X : %d\n", x, i, x_fix, i);
    }
    return 0;
}
