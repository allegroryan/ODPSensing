#include "Enes100.h"

void setup() {
    // Initialize Enes100 library
    // Team Name, Mission Type, Marker ID, TX Pin, RX Pin
    Enes100.begin("ODP", BLACK_BOX, 11, 8, 9);
    Serial.begin(9600);
    // Any other setup code...
}

void loop() {
    // Update the OSV's current location
    Enes100.updateLocation();
    Serial.print("x: ");
    Serial.println(Enes100.location.x);
    Serial.print("y: ");
    Serial.println(Enes100.location.y);
    Serial.print("theta: ");
    Serial.println(Enes100.location.theta);
    if (Enes100.updateLocation()) {
        Enes100.print("OSV is at (");
        Enes100.print(Enes100.location.x);
        Enes100.print(", ");
        Enes100.print(Enes100.location.y);
        Enes100.print(", ");
        Enes100.print(Enes100.location.theta);
        Enes100.println(")");
        Enes100.println("GO ODP!!!!!");
        delay(1000);
    } else {
        // OSV's location was not found
        Enes100.println("404 Not Found");
    }

    // Transmit the coordinate of the black box
    Coordinate blackBox(2.9, 1.4);
    Enes100.mission(blackBox);
}
