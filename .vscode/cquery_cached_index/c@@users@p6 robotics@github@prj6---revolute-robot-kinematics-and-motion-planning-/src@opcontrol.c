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

 //#include"shoulder.c"

void operatorControl() {
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
   }
}
