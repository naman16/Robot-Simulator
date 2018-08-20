/**
 * @file sensor.h
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

#ifndef SRC_SENSOR_H_
#define SRC_SENSOR_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "src/pose.h"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 *
 * @brief The base class from which all Sensors inherit i.e. the parent class
 * from which both the food and the light sensors inherit.
 */


class Sensor {
 public:
  /**
 * @brief Sensor constructor initialized with default values from params.h
 */
  explicit Sensor(Pose pos): sensor_position_(pos), sensor_reading_(0) {}

  virtual ~Sensor() = default;
  /**
  * @brief Getter for sensor position.
  */
  Pose get_sensor_position() { return sensor_position_;}
  /**
  * @brief Setter for sensor position.
  */
  void set_sensor_position(double x, double y) {
    sensor_position_.x = x;
    sensor_position_.y = y;
  }
  /**
  * @brief Setter for sensor position.
  */
  void set_sensor_position(Pose p) {
    sensor_position_.x = p.x;
    sensor_position_.y = p.y;
  }
  /**
  * @brief Getter for sensor reading
  */
  double get_sensor_reading() { return sensor_reading_; }
  /**
  * @brief Setter for sensor reading.
  */
  void set_sensor_reading(double reading) { sensor_reading_ = reading; }
  /**
  * @brief Calculates sensor reading value based on the position of the robot
  * from the entity.
  * @param[in] location_entity The location of the entity for which
  * need to calculate sensor reading.
  */
  virtual void CalculateSensorReading(Pose location_entity) = 0;

 private:
  Pose sensor_position_;
  double sensor_reading_;
};

NAMESPACE_END(csci3081);

#endif  // SRC_SENSOR_H_
