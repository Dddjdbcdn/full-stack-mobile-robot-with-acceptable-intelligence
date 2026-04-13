#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

// Expose target RPM so it can be updated by micro-ROS subscriptions if needed
extern volatile float target_rpm_L;
extern volatile float target_rpm_R;
extern volatile float motor_rpm_L;
extern volatile float motor_rpm_R;
extern volatile float position_L;
extern volatile float position_R;
extern volatile float Kf;
extern volatile float Kp;
extern volatile float Ki;
extern volatile float Kd;

#endif // MOTOR_H