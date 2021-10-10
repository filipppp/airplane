#ifndef AIRPLANE_SENSORS_H
#define AIRPLANE_SENSORS_H

void set_mean_ground();
int setup_sensors(bool save = false);
float get_altitude();
float get_temp();
void write_int(int address, int number);
int read_int(int address);
uint8_t read_battery_voltage();
void set_q(float *q_arr);



#endif //AIRPLANE_SENSORS_H
