//#include "Enes100.h"
//#include "TankSimulation.h"
#include <math.h>
#include <Enes100.h>
//Un Comment obstacle dodging

/*
 * Note: This sketch contains function defintions that may be useful for writing your navigation code. 
 * You are responsible for writing the code to complete the functions and testing it out on the simulator. 
 * If you're looking for an example of using the simulator libraries, go to File->Examples->navigation_example
 * Enes100.location.y
 * Enes100.location.x
 * Enes100.location.theta
 */

 /*
  * Possible Debug Concern priorities:
  * If statements for progress of clockwise movement
  * OR
  * movement as a whole for goals > 3.14 and <-3.14
  * -Max
  */

  /*
   * Mission Notes:
   * Phase 1-
   *  -Center - (0.35,1)
   * Phase 2-
   *  - Sensors: Front[0(Left),2(Right)]
   *             Side[10(Left),4(Right)]
   *             Enes100.readDistanceSensor(int SensorNumber);
   *    Drives down center and dodges obstacles
   *    Obstacle size: 0.2 x 0.5m
   */

   
boolean phase1 = true;
boolean dodgeObstacle1 = false;
//Sonar Values Start:
int trigPin=10;
int echoPin1=2; //Left
int echoPin2=5; //Front Left
int echoPin3=4; //Front Right
int echoPin4=6; //Right
long duration1;
long duration2;
long duration3;
long duration4;
int distance1;
int distance2;
int distance3;
int distance4;
//Sonar Values End

void setup() {
  initialize(); //Sets up motors
  
  Enes100.begin("ODP", BLACK_BOX, 5, 1, 0);
  Enes100.println("-=xxxXXX[Operation Dark Phoenix]XXXxx=- IN DIS BITS:");
  
  /*
  Enes100.println("");
  Enes100.println("__________________");
  Enes100.println("FaceDir Command:");
  faceDir(0);
  */
  
  
  phaseOne();
  phaseTwo();
  
}


void loop() {
  // put your main code here, to run repeatedly:
  
}

void testRun(){
  Enes100.updateLocation();
  Enes100.print("OSV is at (");
  Enes100.print(Enes100.location.x);
  Enes100.print(", ");
  Enes100.print(Enes100.location.y);
  Enes100.print(", ");
  Enes100.print(Enes100.location.theta);
  Enes100.println(")");
  Enes100.print("Left Front Sonar: ");
  Enes100.println(sonarReadDistanceSensor(0));
  Enes100.print("Front Right Sonar: ");
  Enes100.println(sonarReadDistanceSensor(2));
  Enes100.print("Side Right Sonar: ");
  Enes100.println(sonarReadDistanceSensor(4));
  delay(300);
}

//Phase 1 - Center Self
void phaseOne(){
  Enes100.println("-=PHASE 1=-"); 
  if(Enes100.location.y > 1){
    driveDestination(false, 255, 0.2, 1);
  }
  else if(Enes100.location.y < 1){
    driveDestination(false, 255, 0.2, 1);
  }
  else{
    driveDestination(false, 255, 0.2, 1);
  }
  faceDir(0);
  phase1 = false;
}

void phaseTwo(){
  Enes100.println("-=PHASE 2=-"); 
  /*driveDestination(false, 255, 0.9, 1);
  if(dodgeObstacle1){
    Enes100.println("Obstacle Dodged!");
    delay(1000);
    faceDir(0);
    driveDestination(false, 255, 3, 1);
  }
  else{
    Enes100.println("Stop Before Obstacle 1");
    delay(1000);
    faceDir(0);
    driveDestination(true, 255, 1.55, 1);
    if(dodgeObstacle1){
      Enes100.println("Obstacle Dodged!");
      delay(1000);
      faceDir(0);
      driveDestination(false, 255, 3, 1);
    }
    else{
      Enes100.println("Stop Before Obstacle 2");
      delay(1000);
      faceDir(0);
      driveDestination(true, 255, 2.1, 1);
      Enes100.println("Stop Before Obstacle 3");
      delay(1000);
      faceDir(0);
      driveDestination(true, 255, 3, 1);
    }
  }*/
  driveDestination(true, 255, 3, .9);
}

void phaseThree() { // find black box with sonars, orient to face it, and drive towards it
  Enes100.println("-=PHASE 3=-"); 
  float d[30];
  float pi = 3.1415926;
  int r = 0;
  int s = 0;
  float ang;
  Enes100.updateLocation();
  faceDir(pi/2);
  while (s==0){
    int i = 0;
      faceDir(pi/2 + ((i+1)*0.10588));
      d[i] = (sonarReadDistanceSensor(10));
      Enes100.println("");
      Enes100.println("Sonar reading: ");
      Enes100.println(d[i]);
      Enes100.println("");
      if (d[i] <= 2) d[i] = d[i-1];
      /*if (d[i] <= (d[i-1]-8)) {
        r = pi/2 + ((i+1)*0.10588);
        Enes100.println("Found r");
      }*/
      if (d[i+1] >= (d[i]+8)) {
        s = pi/2 + ((i+1)*0.10588);
        Enes100.println("Found s");
        Enes100.print("STOP!-Rotating");
        delay(3000);
      }
      if (s!=0) {
        ang = s;
        Enes100.println("Found ang for Black Box");
        driveBreak();
      }
      delay(500);
    i++;
    if (i >=30) i = 0;
  }
  faceDir(ang);

  Enes100.updateLocation();
  float x,y;
  x = Enes100.location.x + sonarReadDistanceSensor(10)*cos(ang) + 4;
  y = Enes100.location.y + sonarReadDistanceSensor(10)*sin(ang) + 4;
  Enes100.print("(x,y): (");
  Enes100.print(x);
  Enes100.print(",");
  Enes100.print(y);
  Enes100.print(")");

  driveDestination(false,255,x,y);
}

void dodgeObstacle(){
  if(!dodgeObstacle1){
    dodgeObstacle1 = true;
  }
  updateNavigation();
  boolean drivePast = false;
  float sideDistance = sonarReadDistanceSensor(4);
  double startPos [] = {Enes100.location.x, Enes100.location.y, Enes100.location.theta};
  Enes100.println("DODGE OBSTACLE: 1A");
  Enes100.println("DODGE OBSTACLE: DRIVE TEST -------");
  updateNavigation();
  if(Enes100.location.y >= 0.9){
    Enes100.print("Y: ");
    Enes100.print(Enes100.location.y);
    Enes100.print(" ");
    Enes100.println("UPPER HEMISPHERE DODGE");
    driveDestination(false, 255, (Enes100.location.x + 0.85), (Enes100.location.y + 0.7));
  }
  else{
    Enes100.print("Y: ");
    Enes100.print(Enes100.location.y);
    Enes100.print(" ");
    Enes100.println("LOWER HEMISPHERE DODGE");
    driveDestination(false, 255, (Enes100.location.x + 0.85), (Enes100.location.y + 0.8));
  }
  delay(3000);
  updateNavigation();
  Enes100.println("DODGE OBSTACLE: 2-------------------");
  faceDir(0);
  Enes100.println("DODGE OBSTACLE: 2 - Face Right-------------------");
  updateNavigation();
  //driveDestination(false, 255, Enes100.location.x + 0.25, Enes100.location.y);
  updateNavigation();
  Enes100.println("DODGE OBSTACLE: 3-------------------");
  driveDestination(false, 255, (Enes100.location.x + 0.35), startPos[1]);
  updateNavigation();
  faceDir(0);
}


void initialize(){
  //Motors Start:
  pinMode(12, OUTPUT); 
  pinMode(13, OUTPUT); 
  pinMode(9, OUTPUT); //Initiates Brake Channel A pin
  pinMode(8, OUTPUT); //Initiates Brake Channel B pin
  //Motors End

  //Sonar Start:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  pinMode(echoPin4, INPUT);
  //Sonar End
}
void driveDestination(boolean sensing, float defaultOsvSpeed, float x, float y){ //maybe add boolean for gradual speed
  int debugTicker = 0;
  double debug1 = 0;
  double debug2 = 2;
  updateNavigation();
  double osvSpeed = defaultOsvSpeed;
  double startPos [] = {Enes100.location.x, Enes100.location.y, Enes100.location.theta};
  double tAdjacent = x - Enes100.location.x;
  double tOpposite = y - Enes100.location.y;
  float angleChange = atan2(tOpposite, tAdjacent) - startPos[2];
  Enes100.print("arcTan( tOpposite: ");
  Enes100.print(tOpposite);
  Enes100.print("/ tAdjacent:");
  Enes100.print(tAdjacent);
  Enes100.print(") = ");
  Enes100.println(angleChange);
  float distance = sqrt(pow(tAdjacent,2) + pow(tOpposite,2));
  float distanceTraveled = 0;
  float travelPercent = 0;
  Enes100.print("Location: ");
  Enes100.print("(");
  Enes100.print(Enes100.location.x);
  Enes100.print(", ");
  Enes100.print(Enes100.location.y);
  Enes100.println(")");
  Enes100.print("Destination: ");
  Enes100.print("(");
  Enes100.print(x);
  Enes100.print(", ");
  Enes100.print(y);
  Enes100.println(")");
  Enes100.print("Angle Change: ");
  Enes100.println(angleChange);
  Enes100.print("Distance: ");
  Enes100.println(distance);
  Enes100.print("tOpposite: ");
  Enes100.println(tOpposite);
  Enes100.print("tAdjacent: ");
  Enes100.println(tAdjacent);
  Enes100.println(angleChange);
  Enes100.println(angleChange);
  Enes100.println(angleChange);
  faceDir(Enes100.location.theta + angleChange);
  while(distanceTraveled < distance){
    if(sensing == true){
      updateNavigation();
      Enes100.print("Sensor[0]: ");
      int prev1 = sonarReadDistanceSensor(0);
      int prev2 = sonarReadDistanceSensor(4);
      Enes100.println(sonarReadDistanceSensor(0));
      Enes100.print("Sensor[4]: ");
      Enes100.println(sonarReadDistanceSensor(4));
      while(sonarReadDistanceSensor(0) <= 35 && sonarReadDistanceSensor(0) !=0 && Enes100.location.x >= 0.9){
          Enes100.println("Driving Back");
          driveBackward(255);
        }
        while(sonarReadDistanceSensor(4) <= 35 && sonarReadDistanceSensor(0) !=0){
          Enes100.println("Driving Back");
          driveBackward(255);
        }
      if(((sonarReadDistanceSensor(0) <= 45 || sonarReadDistanceSensor(4) <= 45) && (sonarReadDistanceSensor(0) != 0 || sonarReadDistanceSensor(4) != 0)) &&
      ((prev1 <= 45 || prev2 <= 45) && (prev1 != 0 || prev2 != 0))){//&& travelPercent <= 0.33){
        Enes100.println("-------========Collision!!!!==========--------");
        Enes100.print("Sensor[0]: ");
        Enes100.println(sonarReadDistanceSensor(0));
        Enes100.print("Sensor[4]: ");
        Enes100.println(sonarReadDistanceSensor(4));
        while(sonarReadDistanceSensor(0) <= 35 && sonarReadDistanceSensor(0) !=0 && Enes100.location.x >= 0.9){
          Enes100.println("Driving Back");
          driveBackward(255);
        }
        while(sonarReadDistanceSensor(4) <= 35 && sonarReadDistanceSensor(0) !=0 && Enes100.location.x >= 0.9){
          Enes100.println("Driving Back");
          driveBackward(255);
        }
        driveBreak();
        if(Enes100.location.x >= 0.9){
          dodgeObstacle();
        }
        if(dodgeObstacle1){
          break;
        }
        if(debugTicker % 60 == 0){
          debug1 = sonarReadDistanceSensor(0);
        }
        else if(debugTicker % 30 == 0){
          debug2 = sonarReadDistanceSensor(0);
        }
        if(debug1 == debug2){
          Enes100.println("!!!STUCK GLITCH!!!");
          dodgeObstacle();
        }
        osvSpeed = osvSpeed/1.5;
        if(sonarReadDistanceSensor(0) <= 24.5 || sonarReadDistanceSensor(4) <= 24.5){
          Enes100.println("-= DODGING OBSTACLE! =-");
          dodgeObstacle();
        }
      }
      else{
        osvSpeed = defaultOsvSpeed;
      }
    }
    Enes100.print("Driving: ");
    Enes100.print(distanceTraveled);
    Enes100.print("/");
    Enes100.println(distance);
    updateNavigation();
    distanceTraveled = sqrt(pow(floatDiff(startPos[0],Enes100.location.x),2) + pow(floatDiff(startPos[1],Enes100.location.y),2));
    travelPercent = distanceTraveled / distance;
    if(travelPercent <= .9){
      driveForward(osvSpeed);
    }
    else if(travelPercent <= .95){
      driveForward(osvSpeed / 2);
    }
    else{
      driveForward(osvSpeed / 2.5);
    }
  }
  driveForward(0);
  Enes100.print(distance);
  Enes100.print("/");
  Enes100.println(distanceTraveled);
  driveBreak();
  Enes100.println("STOP! - Driving");
}






float floatDiff(float x, float y){ //float difference function
  if(x >= y){
    return x - y;
  }
  else if(x < y){
    return y - x;
  }
}

void faceDir(float dir){
  updateNavigation();
  float rotateRad = rotateRadians(dir);
  float rotateGoal = Enes100.location.theta + rotateRad;
  float rotateStart = Enes100.location.theta;
  float rotateProgress = Enes100.location.theta;
  float rotateDiff = floatDiff(rotateGoal,rotateProgress);
  float percentDone = (abs(abs(rotateStart) + rotateProgress) / rotateDiff);
  Enes100.print("From ");
  Enes100.print(Enes100.location.theta);
  Enes100.print(" To ");
  Enes100.println(rotateGoal);
  Enes100.print("Radian Distance: ");
  Enes100.println(rotateRad);
  //Enes100.println("Radians to turn: " + rotateRad);
  if(rotateDiff <= 0.05){
    //nothing
    Enes100.println("NOT ROTATING, < 0.05");
  }
  else if(rotateRad > 0){
    //rotat counterClockwise
    while(floatDiff(rotateProgress,rotateGoal) > 0.1){
      updateNavigation();
      if(rotateGoal > PI){
        if(Enes100.location.theta >= 0){
          rotateProgress = Enes100.location.theta;
        }
        else{
          rotateProgress = PI + floatDiff(Enes100.location.theta, -PI);
        }
      }
      else{
        rotateProgress = Enes100.location.theta;
      }
      Enes100.print("Rotation Progress: "); 
      Enes100.print(rotateProgress); 
      Enes100.print("/");
      Enes100.println(rotateGoal);
      Enes100.print("Percent: "); //Artificial percent for speed
      percentDone = (floatDiff(rotateProgress, rotateStart) / rotateDiff);
      Enes100.println(percentDone);
      if(percentDone >= 0.95 && percentDone <= 1.05){
        break;
      }
      //ROTATE DIFF
      if(rotateDiff <= 0.1){
        percentDone = .97;
      }
      if(rotateDiff <= 0.3 && percentDone < 0.8){
        percentDone = .8;
      }
      else if(rotateDiff <= 0.1 && percentDone < 0.8){
        percentDone = .9;
      }
      //PERCENT DONES
      if(percentDone <= .8){ //Motor Speeds
        counterClockwise(255);
      }
      else if(percentDone > 0.8 && percentDone < 1.2){
         if(percentDone < 1){
          counterClockwise(250);
         }
         else if(percentDone > 1){
          clockwise(250);
         }
      }
      else if(percentDone >= 1.2){
        clockwise(255);
      }
    }
    driveBreak();
    Enes100.print("STOP! - Rotating");
    delay(1000);
  }
  else if(rotateRad < 0){ //CLOCKWISE ROTATION
    //rotate clockwise
    while(floatDiff(rotateProgress,rotateGoal) > 0.1){
      updateNavigation();
      if(rotateGoal < -PI){
        if(Enes100.location.theta <= 0){
          rotateProgress = Enes100.location.theta;
        }
        else{
          rotateProgress = -PI - floatDiff(Enes100.location.theta, PI);
        }
      }
      else{
        rotateProgress = Enes100.location.theta;
      }
      Enes100.print("Rotation Progress: "); 
      Enes100.print(rotateProgress); 
      Enes100.print("/");
      Enes100.println(rotateGoal);
      Enes100.print("Percent: "); //Artificial percent for speed
      percentDone = (floatDiff(rotateProgress, rotateStart) / rotateDiff);
      Enes100.println(percentDone);
      if(percentDone >= 0.95 && percentDone <= 1.05){
        break;
      }
      //ROTATE DIFF
      if(rotateDiff <= 0.1){
        percentDone = .97;
      }
      if(rotateDiff <= 0.3 && percentDone < 0.8){
        percentDone = .8;
      }
      else if(rotateDiff <= 0.1 && percentDone < 0.8){
        percentDone = .9;
      }
      //PERCENT DONES
      if(percentDone <= .8){ //Motor Speeds
        clockwise(255);
      }
      else if(percentDone > 0.8 && percentDone < 1.2){
         if(percentDone < 1){
          clockwise(250);
         }
         else if(percentDone > 1){
          counterClockwise(250);
         }
      }
      else if(percentDone >= 1.2){
        counterClockwise(255);
      }
    }
    driveBreak();
    Enes100.print("STOP! - Rotating");
    delay(1000);
  }
  else{
    Enes100.print("_____NOT ROTATING!_____");
  }
}

float rotateRadians(float dir){
  if(dir == Enes100.location.theta){
    return 0;
  }
  boolean Q1 [] = {0,0,0}; // [overall, osv.theta, destination]
  boolean Q2 [] = {0,0,0}; // [overall, osv.theta, destination]
  boolean Q3 [] = {0,0,0}; // [overall, osv.theta, destination]
  boolean Q4 [] = {0,0,0}; // [overall, osv.theta, destination]

  //SETS QUADRANT FOR THE OSV THETA
  if(Enes100.location.theta >= 0 && Enes100.location.theta <= PI/2){
    Q1[0] = true;
    Q1[1] = true;
  }
  else if(Enes100.location.theta <= PI && Enes100.location.theta >= PI/2){
    Q2[0] = true;
    Q2[1] = true;
  }
  else if(Enes100.location.theta >= -PI && Enes100.location.theta <= -PI/2){
    Q3[0] = true;
    Q3[1] = true;
  }
  else if(Enes100.location.theta > -PI/2 && Enes100.location.theta < 0){
    Q4[0] = true;
    Q4[1] = true;
  }
  
  //SETS QUADRANT FOR THE DESTINATION
  if(dir >= 0 && dir <= PI/2){
    Q1[0] = true;
    Q1[2] = true;
  }
  else if(dir >= PI/2 && dir <= PI){
    Q2[0] = true;
    Q2[2] = true;
  }
  else if(dir >= -PI && dir <= -PI/2){
    Q3[0] = true;
    Q3[2] = true;
  }
  else if(dir > -PI/2 && dir < 0){
    Q4[0] = true;
    Q4[2] = true;
  }

  Enes100.print("Q1: ");
  Enes100.print(Q1[0]);
  Enes100.print(Q1[1]);
  Enes100.println(Q1[2]);
  
  Enes100.print("Q2: ");
  Enes100.print(Q2[0]);
  Enes100.print(Q2[1]);
  Enes100.println(Q2[2]);
  
  Enes100.print("Q3: ");
  Enes100.print(Q3[0]);
  Enes100.print(Q3[1]);
  Enes100.println(Q3[2]);
  
  Enes100.print("Q4: ");
  Enes100.print(Q4[0]);
  Enes100.print(Q4[1]);
  Enes100.println(Q4[2]);
  
  //Q1
  if(Q1[1] && Q1[2]){
    if(Enes100.location.theta >= dir){
      return -(floatDiff(Enes100.location.theta,dir));
    }
    else{
      return (floatDiff(Enes100.location.theta,dir));
    }
  }
  else if(Q1[0] && Q2[0]){
    if(Q1[1] && Q2[2]){
      return (floatDiff(Enes100.location.theta,dir));
    }
    else{ // Q1[2] && Q2[1]
      return -(floatDiff(Enes100.location.theta,dir));
    }
  }
  else if(Q1[0] && Q3[0]){
    if(Q1[1] && Q3[2]){
      if(floatDiff(Enes100.location.theta,dir) <= PI){
        return -(floatDiff(Enes100.location.theta,dir));
      }
      else{ // rotate 
        return ((PI * 2) - floatDiff(Enes100.location.theta,dir));
      }
    }
    else{ //Q1[2] && Q3[1]
      if(floatDiff(Enes100.location.theta,dir) <= PI){
        return (floatDiff(Enes100.location.theta,dir));
      }
      else{
        return -(floatDiff(Enes100.location.theta,dir));
      }
    }
  }
  else if(Q1[0] && Q4[0]){
    if(Q1[1] && Q4[2]){
      return -(floatDiff(Enes100.location.theta,dir));
    }
    else{ //Q1[2] && Q4[1]
      return (floatDiff(Enes100.location.theta,dir));
    }
  }
  //Q2
  if(Q2[1] && Q2[2]){
    if(Enes100.location.theta >= dir){
      return -(floatDiff(Enes100.location.theta,dir));
    }
    else{ 
      return (floatDiff(Enes100.location.theta,dir));
    }
  }
  else if(Q2[0] && Q3[0]){
    if(Q2[1] && Q3[2]){
      return ((PI*2) - floatDiff(Enes100.location.theta,dir));
    }
    else{ //Q2[2] && Q3[1]
      return -((PI*2) - floatDiff(Enes100.location.theta,dir));
    }
  }
  else if(Q2[0] && Q4[0]){
    if(Q2[1] && Q4[2]){
      if(floatDiff(Enes100.location.theta,dir) <= PI){
        return -(floatDiff(Enes100.location.theta,dir));
      }
      else{
        return ((PI * 2) - floatDiff(Enes100.location.theta,dir));
      }
    }
    else{ //Q2[2] && Q4[1]
      if(floatDiff(Enes100.location.theta,dir) <= PI){
        return (floatDiff(Enes100.location.theta,dir));
      }
      else{
        return -((PI * 2) - floatDiff(Enes100.location.theta,dir));
      }
    }
  }
  //Q3 / Q4
  else if(Q3[1] && Q3[2]){
    if(Enes100.location.theta <= dir){
      return (floatDiff(Enes100.location.theta,dir));
    }
    else{
      return -(floatDiff(Enes100.location.theta,dir));
    }
  }
  else if(Q3[0] && Q4[0]){
    if(Q3[1] && Q4[2]){
      return (floatDiff(Enes100.location.theta,dir));
    }
    else{ //Q3[2] && Q4[1]
      return -(floatDiff(Enes100.location.theta,dir));
    }
  }
  else if(Q4[1] && Q4[2]){
    if(Enes100.location.theta <= dir){
      return (floatDiff(Enes100.location.theta,dir));
    }
    else{
      return -(floatDiff(Enes100.location.theta,dir));
    }
  }
}

void clockwise(int wheelSpeed){
  int goWheelSpeed = 255;
  //Turning RIGHT
  //Motor A forword @ full speed
  digitalWrite(12, LOW); //Establishes forward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, goWheelSpeed);   //Spins the motor on Channel A at full speed
  //Motor B foreword @ full speed
  digitalWrite(13, LOW);  //Establishes backward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, goWheelSpeed);    //Spins the motor on Channel B at full speed
  
}
void counterClockwise(int wheelSpeed){
  int goWheelSpeed = 255;
  //Turning LEFT
  //Motor A forword @ full speed
  digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, goWheelSpeed);   //Spins the motor on Channel A at full speed
  //Motor B foreword @ full speed
  digitalWrite(13, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, goWheelSpeed);    //Spins the motor on Channel B at full speed
}
void driveForward(int wheelSpeed){
  int goWheelSpeed = 255;
  //Forwards
  //Motor A forword @ full speed
  digitalWrite(12, LOW); //Establishes forward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, goWheelSpeed);   //Spins the motor on Channel A at full speed
  //Motor B foreword @ full speed
  digitalWrite(13, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, goWheelSpeed);    //Spins the motor on Channel B at full speed
}
void driveBackward(int wheelSpeed){
  int goWheelSpeed = 255;
  //Forwards
  //Motor A forword @ full speed
  digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, goWheelSpeed);   //Spins the motor on Channel A at full speed
  //Motor B foreword @ full speed
  digitalWrite(13, LOW);  //Establishes backward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, goWheelSpeed);    //Spins the motor on Channel B at full speed
}

void driveBreak(){
  digitalWrite(9, HIGH); //Eengage the Brake for Channel A
  digitalWrite(8, HIGH); //Eengage the Brake for Channel B
}





//sonarReadDistanceSensor(0)
//SONAR FUNCTIONS
int sonarReadDistanceSensor(int sonarNum){
  updateNavigation();
  if(sonarNum == 10){ // back
    clearTrig();
    duration1 = pulseIn(echoPin1, HIGH);
    distance1 = duration1*0.034/2;
    distance1 = map(distance1, 2, 74, 2, 100);
    return distance1;
  }
  if(sonarNum == 0){ // front left
    clearTrig();
    duration2 = pulseIn(echoPin2, HIGH);
    distance2 = duration2*0.034/2;
    distance2 = map(distance2, 2, 74, 2, 100);
    return distance2;
  }
  if(sonarNum == 4){ // front right
    clearTrig();
    duration3 = pulseIn(echoPin3, HIGH);
    distance3 = duration3*0.034/2;
    distance3 = map(distance3, 2, 74, 2, 100);
    return distance3;
  }
  if(sonarNum == 2){ // right
    clearTrig();
    duration4 = pulseIn(echoPin4, HIGH);
    distance4 = duration4*0.034/2;
    distance4 = map(distance4, 2, 74, 2, 100);
    return distance4;
  }
}
void updateNavigation(){
  Enes100.updateLocation();
}
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
    distance1 = map(distance1, 2, 74, 2, 100);
    Serial.print("Distance 1: ");
    Serial.println(distance1);
    }
    if(sonarNum==2){
    duration2 = pulseIn(echoPin2, HIGH);
    distance2 = duration2*0.034/2;
    distance2 = map(distance2, 2, 74, 2, 100);
    Serial.print("Distance 2: ");
    Serial.println(distance2);
    }
    if(sonarNum==3){
    duration3 = pulseIn(echoPin3, HIGH);
    distance3 = duration3*0.034/2;
    distance3 = map(distance3, 2, 74, 2, 100);
    Serial.print("Distance 3: ");
    Serial.println(distance3);
    }
    if(sonarNum==4){
    duration4 = pulseIn(echoPin4, HIGH);
    distance4 = duration4*0.034/2;
    distance4 = map(distance4, 2, 74, 2, 100);
    Serial.print("Distance 4: ");
    Serial.println(distance4);
    }
  }
  //END SONAR FUNCTIONS
