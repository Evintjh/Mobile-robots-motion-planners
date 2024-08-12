#include <data_structure/utils.h>

void copy_differential_drive(
    void *dest, void *src
){

}
void get_differential_drive(
    void *src, void *dest){
    memcpy(dest, src, sizeof(differential_drive_t));
}

void get_differential_drive_config(
    void *src, void *dest){
    memcpy(&((differential_drive_t*) dest)->config,
        &((differential_drive_t*) src)->config, 
        sizeof(differential_drive_config_t));
}

void get_differential_drive_sensor(
    void *src, void *dest){
    memcpy(&((differential_drive_t*) dest)->sensor,
        &((differential_drive_t*) src)->sensor, 
        sizeof(differential_drive_sensor_t));
}

void set_differential_drive_actuation(
    void *src, void *dest){
    memcpy(&((differential_drive_t*) dest)->actuation,
        &((differential_drive_t*) src)->actuation, 
        sizeof(differential_drive_actuation_t));
}