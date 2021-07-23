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
const size_t CONTROL_PAYLOAD_SIZE = 6;
uint8_t* control_payload = new uint8_t[CONTROL_PAYLOAD_SIZE];

/* 5 floats with each 4 bytes */
const size_t SENSOR_PAYLOAD_SIZE = 5 * 4;
union SENSORS {
    float sensor_data[5];
    uint8_t final_payload[SENSOR_PAYLOAD_SIZE];
} sensor_payload;

/* Euler variable */
float eulers_angle[3];
float temp = 0;
float altitude = 0;


/* State handling */
int HC12_state = REST;
uint32_t lastUpdate = 0;
uint32_t last_sensor_broadcast = 0;

void update_controls() {
    int temp_throttle = (int16_t) (control_payload[0] << 8) | control_payload[1];
    if (temp_throttle < 180) throttle = temp_throttle;
    if (temp_throttle < 0) throttle = 0;
    x_joy = (int16_t) (control_payload[2] << 8) | control_payload[3];
    y_joy = (int16_t) (control_payload[4] << 8) | control_payload[5];

    /** Set new gotten controls **/
    set_throttle(throttle);
    set_pitch(x_joy);
}

void basic_setup() {
    Serial.begin(9600);
    Wire.begin();
    Wire.setClock(400000);
}

bool read = false;
int byte_counter = 0;
void handle_control_input(uint8_t input) {
    if (input == '<') {
        set_update_led(HIGH);

        read = true;
    } else if (input == '>') {
        set_update_led(LOW);
        lastUpdate = millis();

        update_controls();
        byte_counter = 0;
        read = false;
    } else if (read) {
        control_payload[byte_counter++] = input;
        if (byte_counter >= CONTROL_PAYLOAD_SIZE) {
            read = false;
            byte_counter = 0;
        }
    }
}

void update_sensor_data() {
    /** Altitude and Temperature Readings **/
    altitude = get_altitude();
    temp = get_temp();

    delay(50);
    float* eulers = get_euler();
    delay(50);
    if (eulers) {
        for (int i = 0; i < 3; i++) {
            eulers_angle[i] = eulers[i] * 180 / M_PI;
        }
    }
}

void send_sensor_payload() {
        Serial.println("SEND");
        sensor_payload.sensor_data[0] = altitude;
        sensor_payload.sensor_data[1] = temp;
        for (int i = 0; i < 3; i++) {
            sensor_payload.sensor_data[i + 2] = eulers_angle[i];
        }

        HC12.print('<');
        HC12.write(sensor_payload.final_payload, 20);
        HC12.print('>');
        last_sensor_broadcast = millis();
}

/* Updates Green Power Led => If no message has been received since more than two seconds,
 * the power led is set to LOW */
void update_power_led() {
    if (millis() - lastUpdate > 2000) {
        buzz();
        set_power_led(LOW);
    } else {
        stop_buzz();
        set_power_led(HIGH);
    }
}


/* SETUP METHOD */
void setup() {
    basic_setup();
    setup_sensors();
    setup_servos();
    setup_leds();

    /* Debug */
    HC12.attachInterrupt(handle_control_input);
    HC12.begin(9600);
    Serial.println("FINISH");
    set_power_led(HIGH);
}

void loop() {
    handle_servo_change();
    update_power_led();

    /** update sensor payload before sending back to controller **/
    send_sensor_payload();

    update_sensor_data();

    Serial.print(throttle);
    Serial.print(";");
    Serial.print(x_joy);
    Serial.print(";");
    Serial.print(y_joy);
    Serial.print(";");
    Serial.print(altitude);
    Serial.print(";");
    Serial.print(temp);
    Serial.print(";");
    Serial.print(eulers_angle[0]);
    Serial.print(";");
    Serial.print(eulers_angle[1]);
    Serial.print(";");
    Serial.println(eulers_angle[2]); // z
}