#include <NeoSWSerial.h>
#include <Adafruit_BMP280.h>
#include "sensors.h"
#include "servos.h"
#include "helpers.h"
#include "../.pio/libdeps/uno/PacketSerial/src/PacketSerial.h"

PacketSerial HC12;

/* Sensor variable */
struct CONTROLS {
    int16_t throttle = 0;
    int16_t x_joy = 512;
    int16_t y_joy = 512;
} controls;

struct SENSORS {
    float temperature = 0;
    float altitude = 0;
    float x_angle = 0;
    float y_angle = 0;
    float z_angle = 0;
    float q[4] = {0,0,0,0};
} sensors;

/* State handling */
uint32_t last_control_update = 0;

void update_sensor_data() {
    /** Altitude and Temperature Readings **/
    sensors.altitude = get_altitude();
    sensors.temperature = get_temp();
    float* euler = get_euler(sensors.q);
    if (euler) {
        sensors.x_angle = euler[0] * 180 / M_PI;
        sensors.y_angle = euler[1] * 180 / M_PI;
        sensors.z_angle = euler[2] * 180 / M_PI;
    }
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
    } else {
        buzz();
        delay(1000);
        stop_buzz();
    }
}

/* SETUP METHOD */
void setup() {
    HC12.begin(9600);
    HC12.setPacketHandler(&onPacketReceived);
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

    /** Incoming Throttle from Controller **/
    HC12.update();

//    Serial.print(sensors.x_angle);
//    Serial.print(";");
//    Serial.print(sensors.x_angle);
//    Serial.print(";");
//    Serial.println(sensors.x_angle);
}