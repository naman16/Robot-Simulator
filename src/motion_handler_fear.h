/**
 * @file motion_handler_fear.h
 *
 * @copyright 2018 3081 Staff, All rights reserved.
 */

#ifndef SRC_MOTION_HANDLER_FEAR_H_
#define SRC_MOTION_HANDLER_FEAR_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "src/common.h"
#include "src/motion_handler.h"
#include "src/motion_handler_robot.h"
#include "src/wheel_velocity.h"
#include "src/robot.h"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @brief Class that implements fear behavior of robot towards light.
 * Fear behavior is implemented with positive, direct sensor-motor connections  
 * meaning the reading from the right sensor impacts the velocity of the right 
 * wheel, and the left sensor impacts the left wheel.
 *
 */

class MotionHandlerFear : public MotionHandlerRobot {
 public:
  explicit MotionHandlerFear(ArenaMobileEntity * ent)
       : MotionHandlerRobot(ent) {}
  MotionHandlerFear(const MotionHandlerFear& other) = default;
  MotionHandlerFear& operator=(const MotionHandlerFear& other) = default;

  /**
  * @brief Method that updates velocity based on Fear behavior of robots.
  * Fear behavior of robot is shown when the robot fears light
  * and it is implemented with positive, direct sensor-motor connections.
  *
  *
  * For this motion, the reading from the right sensor impacts the velocity of
  * the right wheel, and the left sensor impacts the left wheel.
  * A positive connection means that the signal and velocity are positively
  * correlated (i.e. the stronger the signal, the faster the wheel moves.)
  * @param lsreading the Left sensor reading.
  * @param rsreading the right sensor reading.
  * @param v the Current wheel velocity of the robot.
  * @returns The updated wheel velocity of the robot based on the sensor
  * reading.
  */
  WheelVelocity UpdateVelocity(double lsreading, double
    rsreading, WheelVelocity v)override;
};

NAMESPACE_END(csci3081);
#endif  // SRC_MOTION_HANDLER_FEAR_H_
