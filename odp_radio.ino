#include "Enes100.h"

//-=xxxXXX[Operation Dark Phoenix]XXXxx=-

void setup() {
  Enes100.begin("ODP", BLACK_BOX, 8, 9, 8);
  Serial.begin(9600);
}

void loop() {
  Enes100.updateLocation();
  Serial.println(Enes100.location.x);
  Serial.println(Enes100.location.y);
  Serial.println(Enes100.location.theta);
  delay(500);
}
