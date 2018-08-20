/**
 * @file food_sensor.h
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

#ifndef SRC_FOOD_SENSOR_H_
#define SRC_FOOD_SENSOR_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "src/pose.h"
#include "src/sensor.h"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 *
 * @brief Class representing a food sensor for each robot within the Arena.
 * Food sensors based on the robot's distance from the food, calculates a sensor
 * reading value which is used to implement CrossConnectionPositive.
 */

class FoodSensor : public Sensor {
 public:
  /**
 * @brief FoodSensor constructor initialized with default values from params.h
 */
  explicit FoodSensor(Pose pose) : Sensor(pose) {}

  virtual ~FoodSensor() = default;
  /**
  * @brief Calculates sensor reading value based on the position of the robot
  * from the food.
  *
  * @param[in] location_entity Location of the food for which sensor reading
  * is being calculated.
  */
  void CalculateSensorReading(Pose location_entity) override;
};

NAMESPACE_END(csci3081);

#endif  // SRC_FOOD_SENSOR_H_
