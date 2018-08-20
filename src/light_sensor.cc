/**
 * @file light_sensor.cc
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <cmath>
#include <iostream>
#include "src/light_sensor.h"
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
void LightSensor::CalculateSensorReading(Pose location_entity) {
  /* using an exponential function to simulate reading
  * sensor should be saturated when light source close
  * readings are in range 0 to 1000.
  */
  double reading_ = get_sensor_reading();
  double distance_x_ = pow(get_sensor_position().x - location_entity.x, 2);
  double distance_y_ = pow(get_sensor_position().y - location_entity.y, 2);
  double distance_ = pow(distance_x_ + distance_y_, 0.5) - LIGHT_RADIUS;
  double base = pow(base_value_, distance_);
  reading_ += 1200/(base);
  set_sensor_reading(reading_);
if (reading_ > 1000)
    set_sensor_reading(1000);
}

NAMESPACE_END(csci3081);
