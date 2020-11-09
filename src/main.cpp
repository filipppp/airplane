#include <NeoSWSerial.h>
#include <Adafruit_BMP280.h>

#include "sensors.h"
#include "servos.h"
#include "helpers.h"

/* Reciever configuration */
NeoSWSerial HC12(5, 6); // HC-12 TX Pin, HC-12 RX Pin

int16_t throttle = 0;
int16_t x_joy = 512;
int16_t y_joy = 512;

const size_t PAYLOAD_SIZE = 6;
uint8_t* payload = new uint8_t[PAYLOAD_SIZE];
float eulers_angle[3];

int HC12_state = REST;

void update_controls() {
    int temp_throttle = (int16_t) (payload[0] << 8) | payload[1];
    if (temp_throttle < 180) throttle = temp_throttle;
    if (temp_throttle < 0) throttle = 0;
    x_joy = (int16_t) (payload[2] << 8) | payload[3];
    y_joy = (int16_t) (payload[4] << 8) | payload[5];
}

void print_uint8_t(uint8_t n) {
    int i;
    for (i = 8; i >= 0; i--)
        Serial.print((n & (1<<i)) >> i);
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
            HC12.readBytesUntil('>', payload, PAYLOAD_SIZE);
        } else {
            return -1;
        }
    }

    return -1;
}

void setup() {
    basic_setup();
    setup_sensors();
    setup_servos();
    setup_leds();

    /* Debug */
    HC12.begin(9600);
    Serial.println("FINISH");
    show_finish();
}

void loop() {
    handle_servo_change();
    /** Incoming Throttle from Controller **/
    if (HC12_state == REST) {
        HC12_state = RECEIVING;
        update_payload();
        update_controls();

        /** Set new gotten controls **/
        set_throttle(throttle);
        set_pitch(x_joy);

        HC12_state = REST;
    }

    /** Altitude and Temperature Readings **/
    Serial.println(1);
    float alt_read = get_altitude();
    Serial.println(2);

    float temp = get_temp();
    Serial.println(3);

    float* eulers = get_euler();
    Serial.println(4);

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