#include <NeoSWSerial.h>
#include <Adafruit_BMP280.h>

#include "sensors.h"
#include "servos.h"
#include "helpers.h"

/* Reciever configuration */
NeoSWSerial HC12(5, 6); // TX Pin HC12 => RX PIN 5 | TX PIN 6 => RX PIN HC12

/* Aircraft Controls */
int16_t throttle = 0;
int16_t x_joy = 512;
int16_t y_joy = 512;

/* Payload declarations */
const size_t PAYLOAD_SIZE = 6;
uint8_t* payload = new uint8_t[PAYLOAD_SIZE];

/* Euler variable */
float eulers_angle[3];

/* State handling */
int HC12_state = REST;
uint32_t lastUpdate = 0;

void update_controls() {
    int temp_throttle = (int16_t) (payload[0] << 8) | payload[1];
    if (temp_throttle < 180) throttle = temp_throttle;
    if (temp_throttle < 0) throttle = 0;
    x_joy = (int16_t) (payload[2] << 8) | payload[3];
    y_joy = (int16_t) (payload[4] << 8) | payload[5];
}

void basic_setup() {
    Serial.begin(9600);
    Wire.begin();
    Wire.setClock(400000);
}

int update_payload() {
    char in;
    while (HC12.available()) {
        in = HC12.read();
        if (in == '<') {
            HC12_state = RECEIVING;
            HC12.readBytesUntil('>', payload, PAYLOAD_SIZE);
            lastUpdate = millis();
            while (HC12.available()) {
                HC12.read();
            }
        } else {
            return -1;
        }
    }

    return -1;
}

void update_states() {
    if (millis() - lastUpdate > 2000) {
        buzz();
        set_finish(LOW);
    } else {
        stop_buzz();
        set_finish(HIGH);
    }
}

void setup() {
    basic_setup();
    setup_sensors();
    setup_servos();
    setup_leds();

    /* Debug */
    HC12.begin(9600);
    Serial.println("FINISH");
    set_finish(HIGH);
}

void loop() {
    handle_servo_change();
    update_states();
    /** Incoming Throttle from Controller **/
    if (HC12_state == REST) {
        set_update(HIGH);

        update_payload();
        update_controls();

        /** Set new gotten controls **/
        set_throttle(throttle);
        set_pitch(x_joy);

        HC12_state = REST;
        set_update(LOW);
    }

    /** Altitude and Temperature Readings **/
    float alt_read = get_altitude();
    float temp = get_temp();

    delay(50);
    float* eulers = get_euler();
    delay(50);

    if (eulers) {
        for (int i = 0; i < 3; i++) {
            eulers_angle[i] = eulers[i] * 180 / M_PI;
        }
    }

    Serial.print(throttle);
    Serial.print(";");
    Serial.print(x_joy);
    Serial.print(";");
    Serial.print(y_joy);
    Serial.print(";");
    Serial.print(alt_read);
    Serial.print(";");
    Serial.print(temp);
    Serial.print(";");
    Serial.print(eulers_angle[0]);
    Serial.print(";");
    Serial.print(eulers_angle[1]);
    Serial.print(";");
    Serial.println(eulers_angle[2]); // z
}