/**
 * @file motion_handler_exploratory.h
 *
 * @copyright 2018 3081 Staff, All rights reserved.
 */

#ifndef SRC_MOTION_HANDLER_EXPLORATORY_H_
#define SRC_MOTION_HANDLER_EXPLORATORY_H_

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
 * @brief Class that implements exploratory behavior of robot towards light.
 * Exploratory is implemented with negative, crossed sensor-motor connections  
 * meaning the reading from the right sensor impacts the velocity of the left 
 * wheel, and the left sensor impacts the right wheel.
 *
 */

class MotionHandlerExploratory : public MotionHandlerRobot {
 public:
  explicit MotionHandlerExploratory(ArenaMobileEntity * ent)
       : MotionHandlerRobot(ent) {}
  MotionHandlerExploratory(const MotionHandlerExploratory& other) = default;
  MotionHandlerExploratory& operator=(const MotionHandlerExploratory& other)
                                                                  = default;

  /**
  * @brief Method that updates velocity based on Exploratory behavior of robots.
  * Exploratory behavior of robot is shown when the robot explores light
  * and it is implemented with negative, crossed sensor-motor connections.
  *
  *
  * For this motion, the reading from the right sensor impacts the velocity of
  * the left wheel, and the left sensor impacts the right wheel.
  * A negative connection is a negative correlation between the sensor reading
  * and velocity (i.e. the stronger the signal, the slower the wheel moves.)
  * Hence based on the left and right sensor reading, right wheel and left wheel
  * velocity is calculated.
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
#endif  // SRC_MOTION_HANDLER_EXPLORATORY_H_
