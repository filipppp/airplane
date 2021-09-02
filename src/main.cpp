#include <Wire.h>
#include "../.pio/libdeps/megaatmega2560/TinyGPSPlus/src/TinyGPS++.h"

TinyGPSPlus gps;
float lattitude,longitude;

#include "sensors.h"
#include "servos.h"
#include "helpers.h"
#include "../.pio/libdeps/megaatmega2560//PacketSerial/src/PacketSerial.h"

PacketSerial_<COBS, 0, 512> HC12;

/* Sensor variable */
struct CONTROLS {
    int16_t throttle = 0;
    int16_t x_joy = 512;
    int16_t y_joy = 512;
} controls;

struct SENSORS {
    float temperature = 0;
    float altitude = 0;
    float q[4] = {0,0,0,0};
} sensors;

/* State handling */
uint32_t last_control_update = 0;

void update_sensor_data() {
    /** Altitude and Temperature Readings **/
    sensors.altitude = get_altitude();
    sensors.temperature = get_temp();
    refresh_angles(sensors.q);
}

void onPacketReceived(const uint8_t* buffer, size_t size) {
    if (size == sizeof(controls)) {
        set_update_led(HIGH);

        /** Update controls **/
        memcpy(&controls, buffer, size);
        /** Set new gotten controls **/
        set_throttle(controls.throttle);
        set_pitch(controls.x_joy);
        last_control_update = millis();

        /** Send current telemetry back **/
        HC12.send((uint8_t *)&sensors, sizeof(sensors));

        set_update_led(LOW);
    }
//    else {
//        buzz();
//        delay(1000);
//        stop_buzz();
//    }
}

/* SETUP METHOD */
void setup() {
    // Terminal
    Serial.begin(9600);
    // HC-12
    Serial1.begin(9600);
    HC12.setStream(&Serial1);
    HC12.setPacketHandler(&onPacketReceived);
    // GPS
    Serial2.begin(9600);
    Wire.begin();
    Wire.setClock(400000);

    /** Setup sensors etc. **/
    setup_sensors();
    setup_servos();
    setup_leds();

    /* Set Power LED */
    set_power_led(HIGH);
}

void loop() {
    handle_servo_change();
    update_power_led(last_control_update);
    update_sensor_data();

    while (Serial2.available())
    {
        int data = Serial2.read();
        if (gps.encode(data))
        {
            lattitude = (gps.location.lat());
            longitude = (gps.location.lng());
            Serial.print ("lattitude: ");
            Serial.println (lattitude);
            Serial.print ("longitude: ");
            Serial.println (longitude);
        }
    }

    /** Incoming Throttle from Controller **/
    HC12.update();

//    Serial.print(sensors.x_angle);
//    Serial.print(";");
//    Serial.print(sensors.x_angle);
//    Serial.print(";");
//    Serial.println(sensors.x_angle);
}