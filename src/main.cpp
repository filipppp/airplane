#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
//#include "../.pio/libdeps/megaatmega2560/TinyGPSPlus/src/TinyGPS++.h"
//TinyGPSPlus gps;
//float lattitude,longitude;

#include "sensors.h"
#include "servos.h"
#include "helpers.h"
#include "../.pio/libdeps/megaatmega2560/PacketSerial/src/PacketSerial.h"

PacketSerial_<COBS, 0, 512> HC12;

/* Sensor variable */
struct CONTROLS {
    int8_t pitch = 0;
    int8_t roll = 0;
    uint8_t throttle = 0;
    uint8_t buttons = 0; // _ | _ | _ | _ | _ | _ | autopilot | calibrate
} controls;

struct SENSORS {
    float temperature = 0;
    float altitude = 0;
    float q[4] = {0,0,0,0};
    uint8_t voltage = 0;
} sensors;

/* State handling */
uint32_t last_control_update = 0;

void update_sensor_data() {
    /** Altitude and Temperature Readings **/
    sensors.altitude = get_altitude();
    sensors.temperature = get_temp();
    refresh_angles(sensors.q);
    sensors.voltage = read_battery_voltage();
}

/* SETUP METHOD */
void setup() {
    // Terminal
    Serial.begin(9600);
    // HC-12
    Serial1.begin(9600);

    // i2c
    Wire.begin();
    Wire.setClock(400000);
    Wire.setWireTimeout(3000, true);

    /** Setup sensors etc. **/
    setup_sensors();
    setup_servos();
    setup_pin_modes();

    /* Set Power LED */
    set_power_led(HIGH);

    // watchdog
//    wdt_enable(WDTO_500MS);
}

void handle_radio() {
    CONTROLS temp_controls;
    if (Serial1.available() < sizeof(controls)) return;
    size_t read = Serial1.readBytes((uint8_t*) &temp_controls, sizeof(controls));
    if (read == sizeof(controls)) {
        last_control_update = millis();
        set_update_led(HIGH);

        controls.throttle = temp_controls.throttle;
        controls.pitch = temp_controls.pitch;
        controls.roll = temp_controls.roll;
        controls.buttons = temp_controls.buttons;
        if ((controls.buttons & 2) == 2) {
            check_calibration(true);
        } else {
            Serial1.write((uint8_t *) &sensors, sizeof(sensors));
        }
        set_update_led(LOW);
    }
}


void loop() {
    handle_servo_change();
    update_power_led(last_control_update);
    update_sensor_data();
    check_calibration();
    handle_radio();

//    Serial.print(sensors.temperature);
//    Serial.print(";");
//    Serial.print(sensors.altitude);
//    for (int i = 0; i < 4; ++i) {
//        Serial.print(";");
//        Serial.print(sensors.q[i]);
//    }
//    Serial.print(';');
//    Serial.println(millis() - time);
//    wdt_reset();
//    Serial.println(millis() - time);
}