#include <platform_data_structure/kelo_wheel.h>

/*
void configure_kelo_wheel(const char *file_path, 
    kelo_wheel_t *kelo_wheel, int *status){

     // define array 
    int number_of_params = 3;  // Amount of parameters to be read, set by user
    param_array_t param_array[number_of_params];

    int i = 0;  // Keeps tracks of amount of elements inside param_array
 
    // User input where to write data to 
    param_array[i] = (param_array_t) {"kelo/wheel_radius", 
        &(kelo_wheel->wheel_radius), PARAM_TYPE_DOUBLE}; i++;
    param_array[i] = (param_array_t) {"kelo/wheel_track", 
        &(kelo_wheel->wheel_track), PARAM_TYPE_DOUBLE}; i++;
    param_array[i] = (param_array_t) {"kelo/pivot_offset", 
        &(kelo_wheel->pivot_offset), PARAM_TYPE_DOUBLE}; i++;

    // generic reader function 
    read_from_input_file(file_path, param_array, number_of_params, status);       
}
*/
