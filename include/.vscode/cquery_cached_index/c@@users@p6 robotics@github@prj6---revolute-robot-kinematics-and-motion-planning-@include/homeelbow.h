#include"main.h"
void homeElbow(){
  while(digitalRead(LIMIT_SWITCH2) != LOW){
    elbowSet(-40);
  }
  elbowSet(0);
}
