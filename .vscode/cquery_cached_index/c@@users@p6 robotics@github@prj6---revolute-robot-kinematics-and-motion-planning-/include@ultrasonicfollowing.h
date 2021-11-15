#include"main.h"
#include"chassis.h"
void ultrasonicFollowing(){
  Ultrasonic dumbSonar;
  dumbSonar = ultrasonicInit(1,2);
  int goal=75;
  int distance = ultrasonicGet(dumbSonar);
  printf("distance to object: %d", distance);
  if(distance>goal){
    chassisSet(127,127);
  }
  else if(distance<goal){
    chassisSet(-127,-127);
  }
  else{
    chassisSet(0,0);
  }
  delay(50);
}
