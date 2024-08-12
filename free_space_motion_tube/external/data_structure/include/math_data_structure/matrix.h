#ifndef MATRIX_MATH_DATA_STRUCTURE_H
#define MATRIX_MATH_DATA_STRUCTURE_H

typedef struct matrix_s{
    int number_of_columns;
    int number_of_rows;
    double *entries;
}matrix_t;

#endif