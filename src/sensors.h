#ifndef AIRPLANE_SENSORS_H
#define AIRPLANE_SENSORS_H

void set_mean_ground();
int setup_sensors();
float get_altitude();
float get_temp();
void read_mpu();
float* refresh_angles(float* q);


#endif //AIRPLANE_SENSORS_H
