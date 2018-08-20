/**
 * @file motion_handler_robot.cc
 *
 * @copyright 2018 3081 Staff, All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "src/motion_handler_robot.h"
#include "src/motion_handler.h"
#include "src/motion_behavior_differential.h"
#include "src/motion_handler_aggression.h"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
// @TODO add clamped


void MotionHandlerRobot::TurnLeft() {
  set_velocity(
    clamp_vel(get_velocity().left  - get_angle_delta()),
    clamp_vel(get_velocity().right + get_angle_delta()));
}

void MotionHandlerRobot::TurnRight() {
  set_velocity(
    clamp_vel(get_velocity().left  + get_angle_delta()),
    clamp_vel(get_velocity().right - get_angle_delta()));}

void MotionHandlerRobot::IncreaseSpeed() {
  set_velocity(
    clamp_vel(get_velocity().left  + get_speed_delta()),
    clamp_vel(get_velocity().right + get_speed_delta()));
}

void MotionHandlerRobot::DecreaseSpeed() {
  set_velocity(
    clamp_vel(get_velocity().left  - get_speed_delta()),
    clamp_vel(get_velocity().right - get_speed_delta()));
    std::cout << get_velocity().left;
    std::cout << get_velocity().right;
}

double MotionHandlerRobot::clamp_vel(double vel) {
  /* adjusting the velocity of the robot so
  * that it doesn't go backwards and doesn't go
  * beyond maxspeed.
  */
  double clamped = 0.0;
  if (vel <= 0) {
    clamped = 5;
  } else {
    clamped = (vel > get_max_speed()) ?
              get_max_speed():
              vel;
  }
  return clamped;
} /* clamp_vel() */

NAMESPACE_END(csci3081);
