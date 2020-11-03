#ifndef AIRPLANE_SENSORS_H
#define AIRPLANE_SENSORS_H

void set_mean_ground();
int setup_sensors();
float get_altitude();
float get_temp();
float* get_euler();

#endif //AIRPLANE_SENSORS_H
