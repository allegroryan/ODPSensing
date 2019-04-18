#include "Enes100Simulation.h"
#include "TankSimulation.h"
#include <math.h>

/*
 * Note: This sketch contains function defintions that may be useful for writing your navigation code. 
 * You are responsible for writing the code to complete the functions and testing it out on the simulator. 
 * If you're looking for an example of using the simulator libraries, go to File->Examples->navigation_example
 * ENES100.location.y
 * ENES100.location.x
 * ENES100.location.theta
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
   *             Enes100Simulation.readDistanceSensor(int SensorNumber);
   *    Drives down center and dodges obstacles
   *    Obstacle size: 0.2 x 0.5m
   */

   
boolean phase1 = true;


void setup() {
  Enes100.begin("ODP", BLACK_BOX, 5, 9, 8);
  Serial.begin(9600);
  Enes100.println("-=xxxXXX[Operation Dark Phoenix]XXXxx=- IN DIS BITS:");
  
  /*
  Enes100Simulation.println("");
  Enes100Simulation.println("__________________");
  Enes100Simulation.println("FaceDir Command:");
  faceDir(0);
  */
  phaseOne();
  phaseTwo();
  
}


void loop() {
  // put your main code here, to run repeatedly:
  
}


//Phase 1 - Center Self
void phaseOne(){
  Enes100Simulation.println("-=PHASE 1=-"); 
  if(ENES100.location.y > 1){
    driveDestination(false, 255, 0.35, 1);
  }
  else if(ENES100.location.y < 1){
    driveDestination(false, 255, 0.35, 1);
  }
  faceDir(0);
  phase1 = false;
}

void phaseTwo(){
  Enes100Simulation.println("-=PHASE 2=-"); 
  driveDestination(true, 255, 3, 1);
}


void dodgeObstacle(){
  ENES100.updateLocation();
  boolean drivePast = false;
  float sideDistance = Enes100Simulation.readDistanceSensor(4);
  double startPos [] = {ENES100.location.x, ENES100.location.y, ENES100.location.theta};
  Enes100Simulation.println("DODGE OBSTACLE: 1A");
  Enes100Simulation.println("DODGE OBSTACLE: DRIVE TEST -------");
  ENES100.updateLocation();
  driveDestination(false, 200, (ENES100.location.x + 0.3), (ENES100.location.y + 0.6));
  ENES100.updateLocation();
  Enes100Simulation.println("DODGE OBSTACLE: 2-------------------");
  faceDir(0);
  //driveDestination(false, 100, ENES100.location.x+ 0.2, ENES100.location.y);
  Enes100Simulation.println("DODGE OBSTACLE: 2 - Face Right-------------------");
  while(!drivePast){
    driveForward(100);
    if(Enes100Simulation.readDistanceSensor(4) <= 0.3){
      drivePast = true;
    }
  }
  ("DODGE OBSTACLE: 3 DRIVEN PAST, NEXT!-------------------");
  ENES100.updateLocation();
  driveDestination(false, 200, (ENES100.location.x + 0.2), (ENES100.location.y));
  ENES100.updateLocation();
  Enes100Simulation.println("DODGE OBSTACLE: 3-------------------");
  driveDestination(false, 200, (ENES100.location.x + 0.35), startPos[1]);
  ENES100.updateLocation();
  faceDir(0);
}




void driveDestination(boolean sensing, float defaultOsvSpeed, float x, float y){ //maybe add boolean for gradual speed
  int debugTicker = 0;
  double debug1 = 0;
  double debug2 = 2;
  ENES100.updateLocation();
  double osvSpeed = defaultOsvSpeed;
  double startPos [] = {ENES100.location.x, ENES100.location.y, ENES100.location.theta};
  double tAdjacent = x - ENES100.location.x;
  double tOpposite = y - ENES100.location.y;
  float angleChange = atan2(tOpposite, tAdjacent) - startPos[2];
  Enes100Simulation.print("arcTan( tOpposite: ");
  Enes100Simulation.print(tOpposite);
  Enes100Simulation.print("/ tAdjacent:");
  Enes100Simulation.print(tAdjacent);
  Enes100Simulation.print(") = ");
  Enes100Simulation.println(angleChange);
  float distance = sqrt(pow(tAdjacent,2) + pow(tOpposite,2));
  float distanceTraveled = 0;
  float travelPercent = 0;
  Enes100Simulation.print("Location: ");
  Enes100Simulation.print("(");
  Enes100Simulation.print(ENES100.location.x);
  Enes100Simulation.print(", ");
  Enes100Simulation.print(ENES100.location.y);
  Enes100Simulation.println(")");
  Enes100Simulation.print("Destination: ");
  Enes100Simulation.print("(");
  Enes100Simulation.print(x);
  Enes100Simulation.print(", ");
  Enes100Simulation.print(y);
  Enes100Simulation.println(")");
  Enes100Simulation.print("Angle Change: ");
  Enes100Simulation.println(angleChange);
  Enes100Simulation.print("Distance: ");
  Enes100Simulation.println(distance);
  Enes100Simulation.print("tOpposite: ");
  Enes100Simulation.println(tOpposite);
  Enes100Simulation.print("tAdjacent: ");
  Enes100Simulation.println(tAdjacent);
  Enes100Simulation.println(angleChange);
  Enes100Simulation.println(angleChange);
  Enes100Simulation.println(angleChange);
  faceDir(ENES100.location.theta + angleChange);
  while(distanceTraveled < distance){
    if(sensing == true){
      ENES100.updateLocation();
      Enes100Simulation.print("Sensor[0]: ");
      Enes100Simulation.println(Enes100Simulation.readDistanceSensor(0));
      Enes100Simulation.print("Sensor[2]: ");
      Enes100Simulation.println(Enes100Simulation.readDistanceSensor(2));
      if(Enes100Simulation.readDistanceSensor(0) <= 0.4 || Enes100Simulation.readDistanceSensor(2) <= 0.4){
        if(debugTicker % 60 == 0){
          debug1 = Enes100Simulation.readDistanceSensor(0);
        }
        else if(debugTicker % 30 == 0){
          debug2 = Enes100Simulation.readDistanceSensor(0);
        }
        if(debug1 == debug2){
          Enes100Simulation.println("!!!STUCK GLITCH!!!");
          dodgeObstacle();
        }
        osvSpeed = osvSpeed/1.5;
        if(Enes100Simulation.readDistanceSensor(0) <= 0.245 || Enes100Simulation.readDistanceSensor(2) <= 0.245){
          Enes100Simulation.println("-= DODGING OBSTACLE! =-");
          dodgeObstacle();
        }
      }
      else{
        osvSpeed = defaultOsvSpeed;
      }
    }
    Enes100Simulation.print("Driving: ");
    Enes100Simulation.print(distanceTraveled);
    Enes100Simulation.print("/");
    Enes100Simulation.println(distance);
    ENES100.updateLocation();
    distanceTraveled = sqrt(pow(floatDiff(startPos[0],ENES100.location.x),2) + pow(floatDiff(startPos[1],ENES100.location.y),2));
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
  Enes100Simulation.print(distance);
  Enes100Simulation.print("/");
  Enes100Simulation.println(distanceTraveled);
  Enes100Simulation.println("STOP! - Driving");
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
  ENES100.updateLocation();
  float rotateRad = rotateRadians(dir);
  float rotateGoal = ENES100.location.theta + rotateRad;
  float rotateStart = ENES100.location.theta;
  float rotateProgress = ENES100.location.theta;
  float rotateDiff = floatDiff(rotateGoal,rotateProgress);
  float percentDone = (abs(abs(rotateStart) + rotateProgress) / rotateDiff);
  Enes100Simulation.print("From ");
  Enes100Simulation.print(ENES100.location.theta);
  Enes100Simulation.print(" To ");
  Enes100Simulation.println(rotateGoal);
  Enes100Simulation.print("Radian Distance: ");
  Enes100Simulation.println(rotateRad);
  //Enes100Simulation.println("Radians to turn: " + rotateRad);
  if(rotateDiff <= 0.05){
    //nothing
    Enes100Simulation.println("NOT ROTATING, < 0.05");
  }
  else if(rotateRad > 0){
    //rotat counterClockwise
    while(rotateProgress < rotateGoal){
      ENES100.updateLocation();
      if(rotateGoal > PI){
        if(ENES100.location.theta >= 0){
          rotateProgress = ENES100.location.theta;
        }
        else{
          rotateProgress = PI + floatDiff(ENES100.location.theta, -PI);
        }
      }
      else{
        rotateProgress = ENES100.location.theta;
      }
      Enes100Simulation.print("Rotation Progress: "); 
      Enes100Simulation.print(rotateProgress); 
      Enes100Simulation.print("/");
      Enes100Simulation.println(rotateGoal);
      Enes100Simulation.print("Percent: "); //Artificial percent for speed
      percentDone = (floatDiff(rotateProgress, rotateStart) / rotateDiff);
      Enes100Simulation.println(percentDone);
      if(percentDone >= 0.99){
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
      //PERCEBT DONES
      if(percentDone <= .8){ //Motor Speeds
        counterClockwise(100); 
      }
      else if(percentDone <= .95){
        counterClockwise(50);
      }
      else{
        counterClockwise(40);
      }
    }
    Enes100Simulation.println("STOP! - turning");
    counterClockwise(0);
  }
  else if(rotateRad < 0){ //CLOCKWISE ROTATION
    //rotate clockwise
    while(rotateProgress > rotateGoal){
      ENES100.updateLocation();
      if(rotateGoal < -PI){
        if(ENES100.location.theta <= 0){
          rotateProgress = ENES100.location.theta;
        }
        else{
          rotateProgress = -PI - floatDiff(ENES100.location.theta, PI);
        }
      }
      else{
        rotateProgress = ENES100.location.theta;
      }
      Enes100Simulation.print("Rotation Progress: "); 
      Enes100Simulation.print(rotateProgress); 
      Enes100Simulation.print("/");
      Enes100Simulation.println(rotateGoal);
      Enes100Simulation.print("Percent: "); //Artificial percent for speed
      percentDone = (floatDiff(rotateProgress, rotateStart) / rotateDiff);
      Enes100Simulation.println(percentDone);
      if(percentDone >= 0.99){
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
      //PERCEBT DONES
      if(percentDone <= .8){ //Motor Speeds
        clockwise(100); 
      }
      else if(percentDone <= .95){
        clockwise(50);
      }
      else{
        clockwise(5);
      }
    }
    Enes100Simulation.print("STOP! - Rotating");
    clockwise(0);
  }
  else{
    Enes100Simulation.print("_____NOT ROTATING!_____");
  }
}

float rotateRadians(float dir){
  if(dir == ENES100.location.theta){
    return 0;
  }
  boolean Q1 [] = {0,0,0}; // [overall, osv.theta, destination]
  boolean Q2 [] = {0,0,0}; // [overall, osv.theta, destination]
  boolean Q3 [] = {0,0,0}; // [overall, osv.theta, destination]
  boolean Q4 [] = {0,0,0}; // [overall, osv.theta, destination]

  //SETS QUADRANT FOR THE OSV THETA
  if(ENES100.location.theta >= 0 && ENES100.location.theta <= PI/2){
    Q1[0] = true;
    Q1[1] = true;
  }
  else if(ENES100.location.theta <= PI && ENES100.location.theta >= PI/2){
    Q2[0] = true;
    Q2[1] = true;
  }
  else if(ENES100.location.theta >= -PI && ENES100.location.theta <= -PI/2){
    Q3[0] = true;
    Q3[1] = true;
  }
  else if(ENES100.location.theta > -PI/2 && ENES100.location.theta < 0){
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

  Enes100Simulation.print("Q1: ");
  Enes100Simulation.print(Q1[0]);
  Enes100Simulation.print(Q1[1]);
  Enes100Simulation.println(Q1[2]);
  
  Enes100Simulation.print("Q2: ");
  Enes100Simulation.print(Q2[0]);
  Enes100Simulation.print(Q2[1]);
  Enes100Simulation.println(Q2[2]);
  
  Enes100Simulation.print("Q3: ");
  Enes100Simulation.print(Q3[0]);
  Enes100Simulation.print(Q3[1]);
  Enes100Simulation.println(Q3[2]);
  
  Enes100Simulation.print("Q4: ");
  Enes100Simulation.print(Q4[0]);
  Enes100Simulation.print(Q4[1]);
  Enes100Simulation.println(Q4[2]);
  
  //Q1
  if(Q1[1] && Q1[2]){
    if(ENES100.location.theta >= dir){
      return -(floatDiff(ENES100.location.theta,dir));
    }
    else{
      return (floatDiff(ENES100.location.theta,dir));
    }
  }
  else if(Q1[0] && Q2[0]){
    if(Q1[1] && Q2[2]){
      return (floatDiff(ENES100.location.theta,dir));
    }
    else{ // Q1[2] && Q2[1]
      return -(floatDiff(ENES100.location.theta,dir));
    }
  }
  else if(Q1[0] && Q3[0]){
    if(Q1[1] && Q3[2]){
      if(floatDiff(ENES100.location.theta,dir) <= PI){
        return -(floatDiff(ENES100.location.theta,dir));
      }
      else{ // rotate 
        return ((PI * 2) - floatDiff(ENES100.location.theta,dir));
      }
    }
    else{ //Q1[2] && Q3[1]
      if(floatDiff(ENES100.location.theta,dir) <= PI){
        return (floatDiff(ENES100.location.theta,dir));
      }
      else{
        return -(floatDiff(ENES100.location.theta,dir));
      }
    }
  }
  else if(Q1[0] && Q4[0]){
    if(Q1[1] && Q4[2]){
      return -(floatDiff(ENES100.location.theta,dir));
    }
    else{ //Q1[2] && Q4[1]
      return (floatDiff(ENES100.location.theta,dir));
    }
  }
  //Q2
  if(Q2[1] && Q2[2]){
    if(ENES100.location.theta >= dir){
      return -(floatDiff(ENES100.location.theta,dir));
    }
    else{ 
      return (floatDiff(ENES100.location.theta,dir));
    }
  }
  else if(Q2[0] && Q3[0]){
    if(Q2[1] && Q3[2]){
      return ((PI*2) - floatDiff(ENES100.location.theta,dir));
    }
    else{ //Q2[2] && Q3[1]
      return -((PI*2) - floatDiff(ENES100.location.theta,dir));
    }
  }
  else if(Q2[0] && Q4[0]){
    if(Q2[1] && Q4[2]){
      if(floatDiff(ENES100.location.theta,dir) <= PI){
        return -(floatDiff(ENES100.location.theta,dir));
      }
      else{
        return ((PI * 2) - floatDiff(ENES100.location.theta,dir));
      }
    }
    else{ //Q2[2] && Q4[1]
      if(floatDiff(ENES100.location.theta,dir) <= PI){
        return (floatDiff(ENES100.location.theta,dir));
      }
      else{
        return -((PI * 2) - floatDiff(ENES100.location.theta,dir));
      }
    }
  }
  //Q3 / Q4
  else if(Q3[1] && Q3[2]){
    if(ENES100.location.theta <= dir){
      return (floatDiff(ENES100.location.theta,dir));
    }
    else{
      return -(floatDiff(ENES100.location.theta,dir));
    }
  }
  else if(Q3[0] && Q4[0]){
    if(Q3[1] && Q4[2]){
      return (floatDiff(ENES100.location.theta,dir));
    }
    else{ //Q3[2] && Q4[1]
      return -(floatDiff(ENES100.location.theta,dir));
    }
  }
  else if(Q4[1] && Q4[2]){
    if(ENES100.location.theta <= dir){
      return (floatDiff(ENES100.location.theta,dir));
    }
    else{
      return -(floatDiff(ENES100.location.theta,dir));
    }
  }
}

void clockwise(int wheelSpeed){
  //Turning RIGHT
  //Motor A forword @ full speed
  digitalWrite(12, LOW); //Establishes forward direction of Channel A
  analogWrite(3, wheelSpeed);   //Spins the motor on Channel A at full speed
  //Motor B foreword @ full speed
  digitalWrite(13, LOW);  //Establishes backward direction of Channel B
  analogWrite(11, wheelSpeed);    //Spins the motor on Channel B at full speed
}
void counterClockwise(int wheelSpeed){
  //Turning LEFT
  //Motor A forword @ full speed
  digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  analogWrite(3, wheelSpeed);   //Spins the motor on Channel A at full speed
  //Motor B foreword @ full speed
  digitalWrite(13, HIGH);  //Establishes backward direction of Channel B
  analogWrite(11, wheelSpeed);    //Spins the motor on Channel B at full speed
}
void driveForward(int wheelSpeed){
  //Forwards
  //Motor A forword @ full speed
  digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  analogWrite(3, wheelSpeed);   //Spins the motor on Channel A at full speed
  //Motor B foreword @ full speed
  digitalWrite(13, HIGH);  //Establishes backward direction of Channel B
  analogWrite(11, wheelSpeed);    //Spins the motor on Channel B at full speed
}
