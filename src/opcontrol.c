 /** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
 //#include"elbow.c"
 #include"elbow.h"
 #include"shoulder.h"
 #include"math.h"
 #include"homeShoulder.h"
 #include"homeElbow.h"
 #include "position.h"
 #include"claw.h"
 #include"open.h"


 //#include"shoulder.c"

void operatorControl() {
  double goal=50;
  double distance;
  int power;
  int turn;

  analogCalibrate(1);
  analogCalibrate(2);
  analogCalibrate(3);
  Ultrasonic dumbSonar;
  dumbSonar = ultrasonicInit(1,2);
  double x = 30;
  double y = 12.7;
  double l1 = 28.5;
  double l2 = 37.5;
  double D;
  double O2;
  double O1;
  double target=40;
	double error;
	double output;
	double Kp=10;
  double maxOut=127;
	Encoder encoder;
   encoder = encoderInit(QUAD_TOP_PORT, QUAD_BOTTOM_PORT, true);
   double errorS;
   double outputS;
   double KpS=10 ;
   Encoder encoderS;
   encoderS = encoderInit(QUAD_TOP_PORT2, QUAD_BOTTOM_PORT2, true);
   double counts = encoderGet(encoder);
   double countsS = encoderGet(encoderS);
   while(1){
     if(joystickGetDigital(1, 7, JOY_LEFT))
     {
       homeShoulder();
       homeElbow();
       encoderReset(encoder);
       encoderReset(encoderS);

            while(counts<85){
              shoulderSet(-40);
              counts = encoderGet(encoderS);
              delay(20);
             printf("moving to home position shoulder %f \n", countsS);
            }
            shoulderSet(0);
            encoderReset(encoderS);
            while(countsS<160 ){
              elbowSet(40);
              counts = encoderGet(encoder);
              delay(20);
              printf("moving to home position elbow %f \n", counts);
            }
            elbowSet(0);
            delay(20);
            encoderReset(encoder);
            encoderReset(encoderS);

            printf(" shoulder homed %f \n", counts);
            printf("elbow homed %f \n", countsS);


          }
     if(joystickGetDigital(1, 7, JOY_DOWN))
     {
         errorS = target - encoderGet(encoderS);
         outputS = KpS * errorS;
         if(abs(outputS) < maxOut){
           shoulderSet(outputS);
         }
         else{
           shoulderSet(outputS/abs(outputS)*maxOut);
         }
     }
     if(joystickGetDigital(1, 7, JOY_UP))
     {
       encoderReset(encoder);
       encoderReset(encoderS);
       printf("closed loop starting \n");
       while(y>-11)
       {
         D= ((x*x)+(y*y)-(l1*l1)-(l2*l2))/(2*l1*l2);
         O2= atan2(sqrt(1-D*D),D);
         O1= atan2(y,x)+atan2(l2*sin(O2),l1+l2*cos(O2));
         O1 = O1*57.296;
         O2 = O2*57.296;
         O1 = O1*1.6667;
         O2 = O2*2;
         error = O2 - encoderGet(encoder);
         output = Kp * error;
         errorS = O1 - encoderGet(encoderS);
         outputS = KpS * errorS;
         printf("motor powers elbow , shoulder  %f , %f \n",output, outputS);
         printf("angles elbow , shoulder  %f , %f, %f \n",O1, O2, D);

         if(abs(outputS) < maxOut){
           shoulderSet(outputS);
         }
         else{
           shoulderSet(outputS/abs(outputS)*maxOut);
         }
         if(abs(output) < maxOut)
         {
           elbowSet(output);
         }
         else
         {
           elbowSet(output/abs(output)*maxOut);
         }

         if(abs(error) < .1 && abs(errorS) < 0.1){
          shoulderSet(0);
          elbowSet(0);
          delay(2000);
         y=y-2.54;
        }
       }
     }
     if(joystickGetDigital(1, 6, JOY_UP)) {
       shoulderSet(127); // pressing up, so lift should go up
     }
     else if(joystickGetDigital(1, 6, JOY_DOWN)) {
       shoulderSet(-127); // pressing down, so lift should go down
     }
     else {
 			shoulderSet(0); // no buttons are pressed, stop the lift
     }
     if(joystickGetDigital(1, 5, JOY_UP)) {
       elbowSet(127); // pressing up, so lift should go up
     }
     else if(joystickGetDigital(1, 5, JOY_DOWN)) {
       elbowSet(-127); // pressing down, so lift should go down
     }
     else
     {
      elbowSet(0); // no buttons are pressed, stop the lift
     }
     while(joystickGetDigital(1, 7, JOY_RIGHT))
     {
       int count=0;
       double errorU;
       double kpU=1;
       double outputU;
       distance = ultrasonicGet(dumbSonar);
       errorU = goal - ultrasonicGet(dumbSonar);
       outputU = kpU * errorU;
       bool needTurn=false;
       if( 0<ultrasonicGet(dumbSonar) && ultrasonicGet(dumbSonar)<100 ){
         needTurn=false;
       }
       else{
         needTurn=true;
       }
       if(needTurn){
         while(count<50){
           chassisSet(40,0);
           delay(500*count);
           chassisSet(0,0);
           if(0<ultrasonicGet(dumbSonar) && ultrasonicGet(dumbSonar)<100){
             count=51;
             chassisSet(40,0);
             delay(500);
             chassisSet(0,0);
           }
           else {
             chassisSet(-40,0);
             delay(500*(count+1));
              chassisSet(0,0);
             if(0<ultrasonicGet(dumbSonar) && ultrasonicGet(dumbSonar)<100)
             {
               count=51;
               chassisSet(-40,0);
               delay(500);
               chassisSet(0,0);
             }
           }
           count++;
         }
       }
       else {
         if(abs(outputU) < maxOut){
           chassisSet(-1*outputU,(outputU));
         }

         else{
           chassisSet(-1*(outputU/abs(outputU))*maxOut,outputU/abs(outputU)*maxOut);
         }
       }
       delay(100);
       printf("distance to object: %d \n", distance);
     }
     int Ll;
     int Rr;
     int Cc;
     while(joystickGetDigital(1, 8, JOY_LEFT)){

       Ll=analogReadCalibrated(1);
       Rr=analogReadCalibrated(3);
       Cc=analogReadCalibrated(2);
       if(Cc<300&&Rr<300&&Ll<300){
         chassisSet(-40,-40);
       }
       else{
       if(Cc > Ll && Cc > Rr){
         chassisSet(20,-20 );
       }
       else if( Ll < Rr){
         chassisSet(35,35);
       }
       else if(Rr < Ll){
         chassisSet(-35,-35);
       }
      }
    }
    chassisSet(0,0);
    if (joystickGetDigital(1,8,JOY_RIGHT)) {
      double l1 = 10.5;
      double l2 = 13.6;
      double x1 = l1+l2-1;
      double y1 = -2; //-1-1
      double a2;
      double a1;
      bool b1;
      bool b2;
      int error;
      int error2;
      for(int x = x1-1; x >= (x1-10); x--) { // 10 in
        b1= true;
        b2= true;
        a2 = position1(x,y1,l1,l2);
        a1 = position2(a2,x,y1,l1,l2);
        a2 -= a1;
        a1 *= (180/M_PI);
        a2 *= -(180/M_PI);
        while(b1 || b2) {
          error = (int) round((0.6*encoderGet(encoderS) - a1));
          error2 = (int) round((0.5*encoderGet(encoder) - a2));
            if((error < 42) && (error > -42) && b1) {
              motorSet(5,error*12);
                 if((-3 < error) && (error < 3)) {
              b1= false;
            }
          } else if(error >= 42 && b1) {
            motorSet(5,127);
          } else if(error <= -42 && b1) {
            motorSet(5,-127);
          } else {
            motorSet(5,0);
          }

          if((error2 < 42) && (error2 > -42) && b2) {
            motorSet(6,error2*12);
            if((-3 < error2) && (error2 < 3)) {
              b2 = false;
            }
          } else if(error2 >= 42 && b2) {
            motorSet(6,127);
          } else if(error2 <= -42 && b2) {
            motorSet(6,-127);
          } else {
            motorSet(6,0);
          }
          }
        }
      }

     chassisSet(0,0);
     if(joystickGetDigital(1, 8, JOY_UP)) {
       clawSet(127); // pressing up, so lift should go up
     }
     else if(joystickGetDigital(1, 8, JOY_DOWN)) {
       clawSet(-127); // pressing down, so lift should go down
     }
     else {
       clawSet(0); // pressing down, so lift should go down
     }
       if(joystickGetDigital(1, 3, JOY_UP)) {
         openSet(127); // pressing up, so lift should go up
       }
       else if(joystickGetDigital(1, 3, JOY_DOWN)) {
         openSet(-127); // pressing down, so lift should go down
       }
       else {
       openSet(0); // no buttons are pressed, stop the lift
     }
     power = joystickGetAnalog(1, 1); // vertical axis on left joystick
      turn = joystickGetAnalog(1, 2); // horizontal axis on left joystick
  //    counts = encoderGet(encoder);
    //  counts2 = encoderGet(encoder2);
     chassisSet(0.5*(power + turn), 0.5*(power - turn));
   }
}
