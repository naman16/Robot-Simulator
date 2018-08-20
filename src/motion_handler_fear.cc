/**
 * @file motion_handler_fear.cc
 *
 * @copyright 2018 3081 Staff, All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "src/motion_handler_robot.h"
#include "src/motion_behavior_differential.h"
#include "src/motion_handler_fear.h"
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

 /* Fear behavior of robot is shown when the robot is scared of light and it
 * is implemented with positive, direct sensor-motor connections.
 * meaning the reading from the right sensor impacts the velocity of the right
 * wheel, and the left sensor impacts the left wheel.
 * A positive connection means that the signal and velocity are positively
 * correlated (i.e. the stronger the signal, the faster the wheel moves.)
 */
WheelVelocity MotionHandlerFear::UpdateVelocity(double lsreading,
                                         double rsreading, WheelVelocity v) {
  // double k = 0.005;
  v.left = clamp_vel(ROBOT_MAX_SPEED  * 100 * lsreading/0.5);
  v.right = clamp_vel(ROBOT_MAX_SPEED  * 100 * rsreading/0.5);
  return v;
}
NAMESPACE_END(csci3081);
