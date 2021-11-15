#include"main.h"
void ultrasonicFollowing(){
  int goal=75;
  int distance = ultrasonicGet(sonar);
  printf("distance to object: %d", distance);
  if(distance>goal){
    chasssisSet(127,127);
  }
  else if(distance<goal){
    chassisSet(-127,-127);
  }
  else{
    chassisSet(0,0);
  }
  delay(50);
}
