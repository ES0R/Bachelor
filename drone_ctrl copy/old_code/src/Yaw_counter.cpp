/*
 * This block just turn the yaw input into a conter index...
*/

#include "main.h"

float Yaw_counter(float difference) {
  yaw = (difference < -20) ? 1 : difference;
  yaw = (difference > 20) ? -1 : difference;
  total_yaw += yaw;
  return total_yaw;
}
