#include "main.h"
#include "chassis.h"

// Sets the speeds of the left and right wheels of the chassis
void chassisSet(int left, int right) {
  motorSet(1, left);
  motorSet(9, right);
}
