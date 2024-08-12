#include <platform_data_structure/kelo_tricycle.h>
#include <read_file/read_file.h>
/*
void configure_kelo_tricycle(const char *file_path, 
    kelo_tricycle_t *kelo_tricycle, int *status){
     // define array 
    int number_of_params = 3;  // Amount of parameters to be read, set by user
    param_array_t param_array[number_of_params];

    int i = 0;  // Keeps tracks of amount of elements inside param_array
 
    // User input where to write data to 
    param_array[i] = (param_array_t) {"tricycle/wheelbase", 
        &(kelo_tricycle->wheelbase), PARAM_TYPE_DOUBLE}; i++;
    param_array[i] = (param_array_t) {"tricycle/width", 
        &(kelo_tricycle->width), PARAM_TYPE_DOUBLE}; i++;
    param_array[i] = (param_array_t) {"tricycle/length", 
        &(kelo_tricycle->length), PARAM_TYPE_DOUBLE}; i++;

    // generic reader function 
    read_from_input_file(file_path, param_array, number_of_params, status);
}
*/
