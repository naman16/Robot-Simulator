/**
 * @file motion_handler_aggression.cc
 *
 * @copyright 2018 3081 Staff, All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "src/motion_handler_robot.h"
#include "src/motion_behavior_differential.h"
#include "src/motion_handler_aggression.h"
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

/* Aggression behavior of robot is shown when the robot is hungry and it is
* Aggression is implemented with positive, crossed sensor-motor connections.
* This means that there is a positive correlation between the right sensor and
* left wheel, and between the left sensor and right wheel.
* Hence based on the left and right sensor reading, right wheel and left wheel
* velocity is calculated. 
*/
WheelVelocity MotionHandlerAggression::UpdateVelocity(double lsreading,
                                           double rsreading, WheelVelocity v) {
  v.left = clamp_vel(ROBOT_MAX_SPEED  * 100 * rsreading/0.5);
  v.right = clamp_vel(ROBOT_MAX_SPEED * 100 * lsreading/0.5);
  return v;
}
NAMESPACE_END(csci3081);
