#include <NeoSWSerial.h>
#include <Adafruit_BMP280.h>

#include "sensors.h"
#include "servos.h"

/* Reciever configuration */
NeoSWSerial HC12(5, 6); // HC-12 TX Pin, HC-12 RX Pin

//
//void setupSensors(){
//    Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
//    Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
//    Wire.write(0x0); //Setting SLEEP register to 0. (Required; see Note on p. 9)
//    Wire.endTransmission();
//    //Wire.beginTransmission(0x1110110);
//    //Wire.write(0xF5);
//    //Wire.write(0b000 0);
//}

void clearSerial() {
    Serial.write(27);       // ESC command
    Serial.print("[2J");    // clear screen command
    Serial.write(27);
    Serial.print("[H");     // cursor to home command
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
    int throttle;
    while (HC12.available()) {
        throttle = HC12.read();
        set_throttle(throttle);
    }

    /** Altitude and Temperature Readings **/
    float alt_read = get_altitude();
//    Serial.print("Altitude: ");
//    Serial.println(alt_read);
    get_euler();
    delay(200);
}

