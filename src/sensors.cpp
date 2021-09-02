#include <Adafruit_BMP280.h>
#include <Simple_MPU6050.h>
#include "sensors.h"

#define MPU6050_DEFAULT_ADDRESS 0x68
Simple_MPU6050 mpu;
ENABLE_MPU_OVERFLOW_PROTECTION();

Quaternion q;

/** BMP280 Config & Variables **/

const int SAMPLE_SIZE_MEAN_PRESSURE = 500;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
float mean_ground_pa = 0;

/** CALIBRATION FOR BMP280 **/
void set_mean_ground() {
    int i = 0;
    float pressure_sum = 0;
    sensors_event_t pressure_event;
    while (i < SAMPLE_SIZE_MEAN_PRESSURE) {
        bmp_pressure->getEvent(&pressure_event);
        pressure_sum += pressure_event.pressure;
        i++;
        delay(20);
    }
    mean_ground_pa = pressure_sum / SAMPLE_SIZE_MEAN_PRESSURE;
    Serial.print("Mean ground value: ");
    Serial.println(mean_ground_pa);
}

void refresh_data(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
    mpu.GetQuaternion(&q, quat);
}

int setup_sensors() {
    /** Barometer & Temperature Sensor **/
    if (!bmp.begin(0x76)) {
//        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
        /// Add warning
        return -1;
    }
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

    set_mean_ground();

    uint8_t val;
//    Serial.println(F("Start:"));
    #ifdef OFFSETS
//        Serial.println(F("Using Offsets"));
        mpu.SetAddress(MPU6050_DEFAULT_ADDRESS).load_DMP_Image(OFFSETS); // Does it all for you
    #else
        mpu.SetAddress(MPU6050_DEFAULT_ADDRESS).CalibrateMPU().load_DMP_Image();// Does it all for you with Calibration
    #endif
    mpu.on_FIFO(refresh_data);
    return 0;
}

void refresh_angles(float* q_arr) {
    mpu.dmp_read_fifo();
    for (int i = 0; i < 4; ++i) {
        q_arr[0] = q.w;
        q_arr[1] = q.x;
        q_arr[2] = q.y;
        q_arr[3] = q.z;
    }
}

float get_altitude() {
    return bmp.readAltitude(mean_ground_pa);
}

float get_temp() {
    return bmp.readTemperature();
}