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
char* payload = new char[PAYLOAD_SIZE*2];
uint8_t* parsed_payload = new uint8_t[PAYLOAD_SIZE];
float eulers_angle[3];

int HC12_state = REST;

void update_controls() {
    int temp_throttle = (int16_t) (parsed_payload[0] << 8) | parsed_payload[1];
    if (temp_throttle < 180) throttle = temp_throttle;
    if (temp_throttle < 0) throttle = 0;
    x_joy = (int16_t) (parsed_payload[2] << 8) | parsed_payload[3];
    y_joy = (int16_t) (parsed_payload[4] << 8) | parsed_payload[5];
}

void basic_setup() {
    Serial.begin(9600);
    Wire.begin();
    Wire.setClock(400000);
}

int fill_payload() {
    int bytes_written = 0;
    int i = 0;
    char in;

    while (HC12.available()) {
        in = HC12.read();
        if (i == 0) {
            if (in == '<') {
                i++;
                continue;
            } else {
                return -1;
            }
        }

        if (in == '>') {
            payload[bytes_written] = '\0';
            return 0;
        } else {
            payload[bytes_written] = in;
            bytes_written++;
        }
    }
    return -1;
}

void parse_payload() {
//     //memory managment? lol
//    delete[] parsed_payload;
    char* token = strtok(payload, ";");
    parsed_payload[0] = atoi(token);
    for (int i = 1; i < PAYLOAD_SIZE; i++) {
        token = strtok(nullptr, ";");
        parsed_payload[i] = atoi(token);
    }
}

void setup() {
    basic_setup();
//    setup_sensors();
//    setup_servos();

    /* Debug */
    HC12.begin(9600);
    Serial.println("FINISH");
}

void loop() {
    handle_servo_change();
    /** Incoming Throttle from Controller **/

    if (HC12_state == REST) {
        HC12_state = RECEIVING;
        int code = fill_payload();

        if (code == 0) parse_payload();

        HC12_state = REST;
    }
    update_controls();
    set_throttle(throttle);

    /** Altitude and Temperature Readings **/
    float alt_read = get_altitude();
    float temp = get_temp();
    float* eulers = get_euler();
    if (eulers) {
        for (int i = 0; i < 3; i++) {
            eulers_angle[i] = eulers[i] * 180 / M_PI;
        }
    }
//
//    Serial.print(throttle);
//    Serial.print(";");
//    Serial.print(x_joy);
//    Serial.print(";");
//    Serial.print(y_joy);
//    Serial.print(";");
//    Serial.print(alt_read);
//    Serial.print(";");
//    Serial.print(temp);
//    Serial.print(";");
//    Serial.print(eulers_angle[0]);
//    Serial.print(";");
//    Serial.print(eulers_angle[1]);
//    Serial.print(";");
//    Serial.println(eulers_angle[2]); // z
}