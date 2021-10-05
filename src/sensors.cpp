#include <Adafruit_BMP280.h>
#include <Simple_MPU6050.h>
#include <EEPROM.h>
#include "sensors.h"

const uint8_t VOLTAGE_PIN = A0;
const float MAX_VOLTAGE = 12.8;
const float MIN_VOLTAGE = 11.18;
const float R_DIV1 = 9880;
const float R_DIV2 = 4650;

const float ARD_MAX_VOLTAGE = MAX_VOLTAGE * R_DIV2 / (R_DIV1 + R_DIV2);
const float ARD_MIN_VOLTAGE = MIN_VOLTAGE * R_DIV2 / (R_DIV1 + R_DIV2);
const long ANALOG_READING_MAX = round(ARD_MAX_VOLTAGE / 0.0048828125);
const long ANALOG_READING_MIN = round(ARD_MIN_VOLTAGE / 0.0048828125);

#define MPU6050_DEFAULT_ADDRESS 0x68
Simple_MPU6050 mpu;
Quaternion q;

/** BMP280 Config & Variables **/
const int SAMPLE_SIZE_MEAN_PRESSURE = 50;
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
}

void refresh_data(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
    mpu.GetQuaternion(&q, quat);
    Serial.println("test");
//    mpu.GetYawPitchRoll(test, &q);
//    Serial.print(test[0] * 180 / PI);
//    Serial.print(';');
//    Serial.print(test[1] * 180 / PI);
//    Serial.print(';');
//    Serial.println(test[2] * 180 / PI);
}

int setup_sensors(bool save) {
    pinMode(VOLTAGE_PIN, INPUT);

    /** Barometer & Temperature Sensor **/
    while (!bmp.begin(0x76)) {}
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
    if (save) {
        set_mean_ground();
        EEPROM.put(8, mean_ground_pa);
    } else {
        EEPROM.get(8, mean_ground_pa);
    }

    /* MPU6050 */
    mpu.Set_DMP_Output_Rate(DMP_100Hz);
    if (save) {
        mpu.SetAddress(MPU6050_DEFAULT_ADDRESS).CalibrateMPU().load_DMP_Image();// Does it all for you with Calibration
        int16_t offsets[6];
        mpu.GetActiveOffsets(offsets);
        for (int i = 0; i < 6; ++i) {
            write_int(12 + i * 2, offsets[i]);
        }
    } else {
        mpu.SetAddress(MPU6050_DEFAULT_ADDRESS)
        .load_DMP_Image(read_int(12), read_int(14), read_int(16),
                        read_int(18), read_int(20), read_int(22));
    }
    mpu.on_FIFO(refresh_data);
    return 0;
}

void refresh_angles(float *q_arr) {
    mpu.dmp_read_fifo(true);
    for (int i = 0; i < 4; ++i) {
        q_arr[0] = q.w;
        q_arr[1] = q.x;
        q_arr[2] = q.y;
        q_arr[3] = q.z;
    }
}

float get_altitude() {
    return bmp.readAltitude((float) mean_ground_pa);
}

float get_temp() {
    return bmp.readTemperature();
}

void write_int(int address, int number) {
    EEPROM.write(address, number >> 8);
    EEPROM.write(address + 1, number & 0xFF);
}

int read_int(int address) {
    byte byte1 = EEPROM.read(address);
    byte byte2 = EEPROM.read(address + 1);
    return (byte1 << 8) + byte2;
}

uint8_t read_battery_voltage() {
    int reading = analogRead(VOLTAGE_PIN);
    if (reading > ANALOG_READING_MAX) return 100;
    if (reading < ANALOG_READING_MIN) return 0;
    return  (reading - ANALOG_READING_MIN) * 100 / (ANALOG_READING_MAX - ANALOG_READING_MIN);
}
