/*
Working ir code example: https://learn.adafruit.com/ir-sensor/using-an-ir-sensor
Sonar example: https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/
*/
#include <IRremote.h>
#include "Enes100.h"

//IR
int IR_PIN = 7;
IRrecv receiver(IR_PIN);
decode_results irResults;
//END IR

//SONAR CODE:
int trigPin=10;
int echoPin1=2;
int echoPin2=3;
int echoPin3=4;
int echoPin4=5;
long duration1;
long duration2;
long duration3;
long duration4;
int distance1;
int distance2;
int distance3;
int distance4;
//END SONAR

void setup() {
  Serial.begin(9600); //in sonar and IR
  /*
  //SONAR CODE:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  pinMode(echoPin4, INPUT);
  //END SONAR CODE
  */

  //IR CODE
  receiver.enableIRIn();
  //END IR CODE

  
}

void loop() {
  /*
  //SONAR CODE:
  sonarDistance(1);
  sonarDistance(2);
  sonarDistance(3);
  sonarDistance(4);
  //END SONAR
  */

  if(receiver.decode(&irResults)){
    Serial.println(irResults.value, HEX);
    receiver.resume();
    }
}


//SONAR FUNCTIONS
void clearTrig(){
  //Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  //sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  }

void sonarDistance(int sonarNum){
    clearTrig();
    if(sonarNum==1){
    duration1 = pulseIn(echoPin1, HIGH);
    distance1 = duration1*0.034/2;
    Serial.print("Distance 1: ");
    Serial.println(distance1);
    }
    if(sonarNum==2){
    duration2 = pulseIn(echoPin2, HIGH);
    distance2 = duration2*0.034/2;
    Serial.print("Distance 2: ");
    Serial.println(distance2);
    }
    if(sonarNum==3){
    duration3 = pulseIn(echoPin3, HIGH);
    distance3 = duration3*0.034/2;
    Serial.print("Distance 3: ");
    Serial.println(distance3);
    }
    if(sonarNum==4){
    duration4 = pulseIn(echoPin4, HIGH);
    distance4 = duration4*0.034/2;
    Serial.print("Distance 4: ");
    Serial.println(distance4);
    }
  }
