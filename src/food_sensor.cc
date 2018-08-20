/**
 * @file food_sensor.cc
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <cmath>
#include "src/food_sensor.h"
#include "src/params.h"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void FoodSensor::CalculateSensorReading(Pose location_entity) {
  /* Calculates the food sensor reading based on the distance of the robot
  * from the food entity.
  */
  double reading_ = get_sensor_reading();
  double distance_x_ = pow(get_sensor_position().x - location_entity.x, 2);
  double distance_y_ = pow(get_sensor_position().y - location_entity.y, 2);
  double distance_ = pow(distance_x_ + distance_y_, 0.5) - FOOD_RADIUS;
  reading_ += 1200/(pow(1.08, distance_));
  set_sensor_reading(reading_);
  if (reading_ > 1000)
    set_sensor_reading(1000);
}

NAMESPACE_END(csci3081);
