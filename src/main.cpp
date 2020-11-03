#include <NeoSWSerial.h>
#include <Adafruit_BMP280.h>

#include "sensors.h"
#include "servos.h"

/* Reciever configuration */
NeoSWSerial HC12(5, 6); // HC-12 TX Pin, HC-12 RX Pin

uint8_t throttle = 0;
uint8_t x_joy = 512;
uint8_t y_joy = 512;
uint8_t* payload;

void update_controls() {
    throttle = payload[0];
    x_joy = payload[1];
    y_joy = payload[2];
}

void basic_setup() {
    Serial.begin(9600);
    Wire.begin();
    Wire.setClock(400000);
}

void setup() {
    basic_setup();
    setup_sensors();
    setup_servos();

    /* Debug */
    HC12.begin(9600);
}

void loop() {
    handle_servo_change();

    /** Incoming Throttle from Controller **/
    while (HC12.available()) {
        HC12.readBytes(payload, 3);
        update_controls();
        set_throttle(throttle);
    }

    /** Altitude and Temperature Readings **/
    float alt_read = get_altitude();
//    Serial.print("Altitude: ");
//    Serial.println(alt_read);
    get_euler();
    delay(200);
}

