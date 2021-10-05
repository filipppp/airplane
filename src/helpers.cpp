//
// Created by filip on 07.11.20.
//

#include <HID.h>
#include <avr/wdt.h>
#include "helpers.h"
#include "sensors.h"

const uint8_t BUZZER_PIN = 3;
const uint8_t POWER_PIN = 5; // GREEN
const uint8_t UPDATE_PIN = 4; // YELLOW
const uint8_t CALIBRATE_PIN = 28;
const uint8_t SET_PIN = 12;


void setup_pin_modes() {
    pinMode(POWER_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(UPDATE_PIN, OUTPUT);
    pinMode(SET_PIN, OUTPUT);
    digitalWrite(SET_PIN, HIGH);
    pinMode(CALIBRATE_PIN, INPUT_PULLUP);
}

void set_power_led(int logic_level) {
    digitalWrite(POWER_PIN, logic_level);
}

void set_update_led(int logic_level) {
    digitalWrite(UPDATE_PIN, logic_level);
}

void buzz() {
//    tone(BUZZER_PIN, 500);
}

void stop_buzz() {
    noTone(BUZZER_PIN);
}

/* Updates Green Power Led => If no message has been received since more than two seconds,
 * the power led is set to LOW */
void update_power_led(uint32_t last_update) {
    if (millis() - last_update > 2000) {
        buzz();
        set_power_led(LOW);
    } else {
        stop_buzz();
        set_power_led(HIGH);
    }
}

void check_calibration(bool force) {
    if (digitalRead(CALIBRATE_PIN) == LOW || force) {
//        wdt_disable();
        setup_sensors(true);
//        wdt_enable(WDTO_60MS);
    }
}