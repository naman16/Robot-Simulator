/**
 * @file motion_handler_exploratory.cc
 *
 * @copyright 2018 3081 Staff, All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "src/motion_handler_robot.h"
#include "src/motion_behavior_differential.h"
#include "src/motion_handler_exploratory.h"
#include "src/motion_handler.h"
#include "src/wheel_velocity.h"
#include "src/robot.h"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

 /* Exploratory behavior of robot is shown when the robot explores light and it
 * is implemented with negative, crossed sensor-motor connections,
 * meaning the reading from the right sensor impacts the velocity of the left
 * wheel, and the left sensor impacts the right wheel.
 * A negative connection is a negative correlation between the sensor reading
 * and velocity (i.e. the stronger the signal, the slower the wheel moves.)
 * Hence based on the left and right sensor reading, right wheel and left wheel
 * velocity is calculated.
 */
WheelVelocity MotionHandlerExploratory::UpdateVelocity(double lsreading,
                                         double rsreading, WheelVelocity v) {
  // double k = 0.0005;
  v.left = clamp_vel(ROBOT_MAX_SPEED * (1-rsreading/0.5));
  v.right = clamp_vel(ROBOT_MAX_SPEED * (1-lsreading/0.5));
  return v;
}
NAMESPACE_END(csci3081);
