/**
 * @file motion_handler_robot.h
 *
 * @copyright 2018 3081 Staff, All rights reserved.
 */

#ifndef SRC_MOTION_HANDLER_ROBOT_H_
#define SRC_MOTION_HANDLER_ROBOT_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <cassert>
#include <iostream>
#include "src/common.h"
#include "src/motion_handler.h"
#include "src/sensor_touch.h"
#include "src/communication.h"
#include "src/wheel_velocity.h"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);
// class MotionHandlerFear;
// class MotionHandlerExploratory;
/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @brief Class managing a Robot's speed and heading angle
 * based on distance from food and light. Towards, light the robot can have
 * fear or exploratory behavior. Whereas towards, food, the robot exhibits
 * aggression.
 *
 */

class MotionHandlerRobot : public MotionHandler {
 public:
  explicit MotionHandlerRobot(ArenaMobileEntity * ent)
      : MotionHandler(ent) {}
  MotionHandlerRobot(const MotionHandlerRobot& other) = default;
  MotionHandlerRobot& operator=(const MotionHandlerRobot& other) = default;
  /**
  * @brief Update the speed and the pose angle according to the sensor readings.
  *
  * Currently does not change speed.
  *
  * @param[in] pose The current pose.
  * @param[in] st A SensorTouch to be read.
  */
  virtual ~MotionHandlerRobot() = default;
  /**
   * @brief Increase the overall speed of the entity by speed_delta.
   */
  void IncreaseSpeed() override;

  /**
   * @brief Decrease the overall speed of the entity by speed_delta.
   */
  void DecreaseSpeed() override;

  /**
   * @brief Turn the entity to the right by angle_delta (in degrees?)
   */
  void TurnRight() override;

  /**
   * @brief Turn the entity to the left by angle_delta (in degrees?)
   */
  void TurnLeft() override;

  /**
  * @brief Updates the velocity of the robot based on its behavior towards
  * the entity. If the entity is light, DirectConnectionPositive if robot fears
  * light or CrossConnectionNegative is implemented if the robot explores light.
  * If the entity is food, CrossConnectionPositive is implemented.
  * @param lsreading Left sensor reading
  * @param rsreading right sensor reading.
  * @param v the current wheel velocity of the robot.
  *
  * @returns The updated wheel velocity based on the sensor readings.
  */
  virtual WheelVelocity UpdateVelocity(__unused double lsreading,
      __unused double rsreading, __unused WheelVelocity v)
      { return get_velocity();}

  /**
   * @brief Makes sure that the velocity for the robot doesn't go beyond
   * maxspeed and also doesn't become negative.
   */
  double clamp_vel(double vel);

 private:
  // MotionHandlerAggression motion_handler_aggression_;
};

NAMESPACE_END(csci3081);

#endif  // SRC_MOTION_HANDLER_ROBOT_H_
