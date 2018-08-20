/**
 * @file sensor_light_unittest.cc
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */




/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <gtest/gtest.h>
#include "src/arena.h"
#include "src/arena_params.h"
#include "src/params.h"
#include "src/arena_entity.h"
#include "src/light.h"
#include "src/light_sensor.h"
#include <cmath>
#ifdef LIGHTSENSOR_TESTS

/******************************************************
* TEST FEATURE SetUp
*******************************************************/

class LightSensorTests : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Initialize Arena for Collections of TESTS
    lightsensor = new csci3081::LightSensor({200, 200});
    lightsensor->set_base_value(1.08);
  }
  double SensorReading(csci3081::Pose location_entity) {
    // using an exponential function to simulate reading
    // sensor should be saturated when light source close
    // readings are in range 0 to 1000
    double reading = lightsensor->get_sensor_reading();
    double distance_x = pow(lightsensor->get_sensor_position().x -
    location_entity.x, 2);
    double distance_y = pow(lightsensor->get_sensor_position().y -
    location_entity.y, 2);
    double distance = pow(distance_x + distance_y, 0.5) - LIGHT_RADIUS;
    float base_value = 1.08;
    reading = 1200/(pow(base_value, distance));
    if (reading > 1000)
    reading = 1000;
    return reading;
  }

  // std::vector<csci3081::ArenaEntity*> lights;
  // std::vector<csci3081::ArenaEntity*> entities;
  // csci3081::Arena * arena;
  csci3081::LightSensor * lightsensor;
};

/*******************************************************************************
 * Test Cases
 ******************************************************************************/
// Testing the constructor
TEST_F(LightSensorTests, Constructor) {
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_position().x, 200)
  << " Fail : LightSensor constructor initialization for x position.";
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_position().y, 200)
  << " Fail : LightSensor constructor initialization for y position.";
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), 0.0)
  << " Fail : LightSensor constructor initialization for sensorreading.";
}
// Testing the Getter and Setter for sensor reading
TEST_F(LightSensorTests, GetSetSensorReading) {
  lightsensor->set_sensor_reading(00.0001);
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), 00.0001)
  << " Fail: Getters and Setters for sensor reading not working correctly";
  lightsensor->set_sensor_reading(100);
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), 100)
  << " Fail: Getters and Setters for sensor reading not working correctly";
  lightsensor->set_sensor_reading(45);
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), 45)
  << " Fail: Getters and Setters for sensor reading not working correctly";
}

// Testing the getter and setter for sensor position
TEST_F(LightSensorTests, GetSetSensorPosition) {
  lightsensor->set_sensor_position(200.0, 120.4);
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_position().x, 200.0)
  << " Fail: Getters and Setters for sensor position not working correctly";
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_position().y, 120.4)
  << " Fail: Getters and Setters for sensor position not working correctly";
  lightsensor->set_sensor_position(300.2, 132.4);
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_position().x, 300.2)
  << " Fail: Getters and Setters for sensor position not working correctly";
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_position().y, 132.4)
  << " Fail: Getters and Setters for sensor position not working correctly";
  lightsensor->set_sensor_position(0, 0);
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_position().x, 0)
  << " Fail: Getters and Setters for sensor position not working correctly";
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_position().y, 0)
  << " Fail: Getters and Setters for sensor position not working correctly";
}

// Assuming the base value for light sensor calculation is 1.08
TEST_F(LightSensorTests, SensorReadingCalculation) {
  lightsensor->set_sensor_reading(0.0);
  lightsensor->set_sensor_position(200, 200);
  // Assuming that there is only one light at (220,220)
  // Calculating sensor value when light is close to sensor
  lightsensor->CalculateSensorReading({220, 220});
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), SensorReading({220, 220}))
  << " Fail for 1 light that is close";
  lightsensor->set_sensor_reading(0.0);
  // Assuming that there is only one light at (800,800)
  // Calculating sensor value when light is far from sensor
  lightsensor->CalculateSensorReading({800, 800});
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), SensorReading({800, 800}))
  << " Fail for 1 light that is far";
  lightsensor->set_sensor_reading(0.0);
  // Checking for the case when the sensor reading becomes greater than 1000/MAX
  lightsensor->CalculateSensorReading({220, 220});
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), 1000)
  << "Fail : reading should be 1000 for this case";
  // Checking for the case when the sensor reading becomes very small and almost
  // close to 0
  lightsensor->set_sensor_reading(0.0);
  lightsensor->CalculateSensorReading({1200,1200});
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), SensorReading({1200,1200}))
  << "Fail : reading should be very small, almost close to 0";
  // Checking for boundary condition when 2 lights are really far from the sensor
  lightsensor->set_sensor_reading(0.0);
  // Assuming that there are 2 lights at positions (1000,1000) and (960,990)
  lightsensor->CalculateSensorReading({1000, 1000});
  lightsensor->CalculateSensorReading({960, 990});
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(),  SensorReading({1000, 1000}) +
  SensorReading({960, 990})) << " Fail for 2 lights minimum boundary condition";
  // Checking for boundary condition when 2 lights are really close to the sensor
  lightsensor->set_sensor_reading(0.0);
  // Assuming that there are 2 lights at positions (215,215) and (225,225)
  lightsensor->CalculateSensorReading({215, 215});
  lightsensor->CalculateSensorReading({225, 225});
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), ((SensorReading({215, 215})+
  SensorReading({225, 225}))>1000) ? 1000:SensorReading({215, 215})+SensorReading({225, 225}))
  << " Fail for 2 lights maximum boundary condition";
  // Checking for random positions now, for 2,3,4,5 lights
  lightsensor->set_sensor_reading(0.0);
  // Assuming that there are 2 lights at positions (100,100) and (400,400)
  lightsensor->CalculateSensorReading({100, 100});
  lightsensor->CalculateSensorReading({400, 400});
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), SensorReading({100, 100}) +
  SensorReading({400, 400})) << " Fail for 2 lights";
  lightsensor->set_sensor_reading(0.0);
  // Assuming that there are 3 lights at positions (500,800),(400,100),(300,700)
  lightsensor->CalculateSensorReading({500, 800});
  lightsensor->CalculateSensorReading({400, 100});
  lightsensor->CalculateSensorReading({300, 700});
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), SensorReading({500, 800}) +
  SensorReading({400, 100}) + SensorReading({300, 700}))
  << " Fail for 3 lights";
  lightsensor->set_sensor_reading(0.0);
  // 4 lights at positions (100,100), (400,400), (300,300), (275,275)
  lightsensor->CalculateSensorReading({100, 100});
  lightsensor->CalculateSensorReading({400, 400});
  lightsensor->CalculateSensorReading({300, 300});
  lightsensor->CalculateSensorReading({275, 275});
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), SensorReading({100, 100})
  + SensorReading({400, 400}) + SensorReading({300, 300})
  + SensorReading({275, 275})) << " Fail for 4 lights";
  lightsensor->set_sensor_reading(0.0);
  // 5 lights at positions (20,30),(1000,600),(400,300),(275,275),(999,111)
  lightsensor->CalculateSensorReading({20, 30});
  lightsensor->CalculateSensorReading({1000, 600});
  lightsensor->CalculateSensorReading({400, 300});
  lightsensor->CalculateSensorReading({275, 275});
  lightsensor->CalculateSensorReading({999, 111});
  EXPECT_DOUBLE_EQ(lightsensor->get_sensor_reading(), SensorReading({20, 30})
  + SensorReading({1000, 600}) + SensorReading({400, 300})
  + SensorReading({275, 275}) + SensorReading({999, 111}))
  << " Fail for 5 lights";
}


#endif /* LIGHTSENSOR_TESTS */
