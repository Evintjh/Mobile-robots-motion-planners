#ifndef QUADRATIC_EQUATION_H
#define QUADRATIC_EQUATION_H

#include <math_data_structure/complex_number.h>


typedef struct quadratic_equation_s{
    double a, b, c;  ///< ax^2 + bx + c = 0
    double delta;  ///< delta = b^2 - 4ac
    complex_number_t roots[2];  
}quadratic_equation_t;

#endif