/**
 * @file motion_handler_aggression.h
 *
 * @copyright 2018 3081 Staff, All rights reserved.
 */

#ifndef SRC_MOTION_HANDLER_AGGRESSION_H_
#define SRC_MOTION_HANDLER_AGGRESSION_H_

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
 * @brief Class that implements aggressive behavior of robot towards food.
 * All robots should exhibit aggression towards food when they are hungry.
 * Aggression is implemented with positive, crossed sensor-motor connections.
 * This means that there is a positive correlation between
 * the right sensor and left wheel, and between the left sensor and right wheel.
 *
 */

class MotionHandlerAggression : public MotionHandlerRobot {
 public:
  explicit MotionHandlerAggression(ArenaMobileEntity * ent)
       : MotionHandlerRobot(ent) {}
  MotionHandlerAggression(const MotionHandlerAggression& other) = default;
  MotionHandlerAggression& operator=(const MotionHandlerAggression& other)
                                                                = default;
  /**
  * @brief Method that updates velocity based on Aggression behavior of robots.
  * Aggression behavior of robot is shown when the robot is hungry and
  * Aggression is implemented with positive, crossed sensor-motor connections.
  *
  * This means that there is a positive correlation between the right sensor and
  * left wheel, and between the left sensor and right wheel.
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
#endif  // SRC_MOTION_HANDLER_AGGRESSION_H_
