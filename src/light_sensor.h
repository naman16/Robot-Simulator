/**
 * @file light_sensor.h
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

#ifndef SRC_LIGHT_SENSOR_H_
#define SRC_LIGHT_SENSOR_H_

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
 * @brief Class representing a light sensor for each robot within the Arena.
 * Light sensors based on the robot's distance from the light, calculates a sensor
 * reading value which influences robot motion.
 */

class LightSensor : public Sensor {
 public:
 /**
 * @brief LightSensor constructor initialized with default values from params.h
 */
  explicit LightSensor(Pose pose) : Sensor(pose) {}

  virtual ~LightSensor() = default;
  /**
  * @brief Calculates the light sensor's reading based on the distance of the
  * light from the robot.
  *
  * @param[in] location_entity The position of the light source for which
  * calculating sensor reading value.
  */
  void CalculateSensorReading(Pose location_entity) override;

  /**
  * @brief Setter for the base value for light sensor calculation as provided
  * by the user.
  *
  * @param[in] lsensor_base The base value for light sensor calculation.
  */
  void set_base_value(float lsensor_base) {
    base_value_ = lsensor_base;
  }

 private:
  // Variable to store the base value for light sensor calculation as provided
  // the user.
  float base_value_ = 0.0;
};

NAMESPACE_END(csci3081);

#endif  // SRC_LIGHT_SENSOR_H_
