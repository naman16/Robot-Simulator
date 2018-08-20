/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <gtest/gtest.h>
#include "src/arena.h"
#include "src/arena_params.h"
#include "src/params.h"
#include "src/arena_entity.h"
#include "src/light_sensor.h"
#include <cmath>
#include "src/robot.h"
#include "src/motion_handler_robot.h"
#include "src/wheel_velocity.h"
#include "src/motion_handler_fear.h"
#include "src/motion_handler_exploratory.h"
#include "src/arena_mobile_entity.h"
#include "src/entity_type.h"
#ifdef MOTIONHANDLER_TESTS

/******************************************************
* TEST FEATURE SetUp
*******************************************************/
class UpdateVelocityTest : public ::testing::Test {

protected:
  virtual void SetUp() {
    // Initialize Arena for Collections of TESTS
    //csci3081::arena_params aparams;
    //arena = new csci3081::Arena(&aparams);
    //robot = arena-> robot();
    //robots = arena->robot();
    //robot_simulators = arena->robot();
    //setting the position of the 2 light sensors for the robot
    robot = new csci3081::Robot(0);
    robot_simulator = new csci3081::Robot(0);
    lightsensorleft = new csci3081::LightSensor(robot->get_left_light_sensor()->
                                        get_sensor_position());
    lightsensorright = new csci3081::LightSensor(robot->get_right_light_sensor()->
                                        get_sensor_position());
    lightsensorleft->set_base_value(1.08);
    robot->get_right_light_sensor()->set_base_value(1.08);
    robot->get_left_light_sensor()->set_base_value(1.08);
    lightsensorright->set_base_value(1.08);
    robot->get_motion_handler()->set_velocity(5,5);
    robot_simulator->get_motion_handler()->set_velocity(5,5);
  }
// flag here is used as a means to determine which behavior has to be
// implemented, 1 means fear, 2 means exploratory.

// function that does the velocity calculation that we are checking against.
  void VelocityWheel(int flag, double leftreading, double rightreading) {
    if (flag==1) { // checking for Direct Connection Positive.
	  // robot fear behavior towards light.
      robot_simulator->get_motion_handler()->set_velocity(
        clamp_vel(ROBOT_MAX_SPEED  * 100 * leftreading/0.5),
        clamp_vel(ROBOT_MAX_SPEED  * 100 * rightreading/0.5));
      return;
    } else { //CrossConnectionNegative implementing
      // robot exploratory behavior towards light
      robot_simulator->get_motion_handler()->set_velocity(
       clamp_vel(ROBOT_MAX_SPEED * (1-rightreading/0.5)),
       clamp_vel(ROBOT_MAX_SPEED * (1-leftreading/0.5)));
      return;
    }
  }

// function that ensures that velocity is never less than 0 and never
// greater than 10.
  double clamp_vel(double vel) {
    double clamped = 0.0;
    if (vel > 0) {
      clamped = (vel > 10) ? 10 : vel;
    } else {
      clamped = 5;
    }
    return clamped;
  } /* clamp_vel() */


  // csci3081::Arena * arena;
  // std::vector<csci3081::Robot*> robots;
  csci3081::Robot * robot;
  csci3081::LightSensor * lightsensorleft;
  csci3081::LightSensor * lightsensorright;
  // robot instance to help in calculating test values
  // to check against
  //std::vector<csci3081::Robot*> robot_simulators;
  csci3081::Robot * robot_simulator;
};

/*******************************************************************************
 * Test Cases
 ******************************************************************************/

// before each set of tests, setting the robot velocity to 5.
// leftreading is the left sensor reading
// rightreading is the right sensor reading
// This method tests Robot's fear behavior towards light.
TEST_F(UpdateVelocityTest, PositiveDirectConnection) {
  // Imagine that there is one light at (100,100)
  robot->set_behavior_flag(1);
  lightsensorleft->set_sensor_reading(0);
  lightsensorright->set_sensor_reading(0);
  robot->get_left_light_sensor()->set_sensor_reading(0.0);
  robot->get_right_light_sensor()->set_sensor_reading(0.0);
  robot->get_motion_handler()->set_velocity(5, 5);
  robot_simulator->get_motion_handler()->set_velocity(5, 5);
  csci3081::Light *light1 = new csci3081::Light();
  light1->set_position(100,100);
  robot->RobotDecideMotion(light1->get_type(), light1);
  lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                              get_sensor_position());
  lightsensorleft->CalculateSensorReading({100,100});
  lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                              get_sensor_position());
  lightsensorright->CalculateSensorReading({100,100});
  double leftreading = lightsensorleft->get_sensor_reading();
  double rightreading = lightsensorright->get_sensor_reading();
  VelocityWheel(1, leftreading, rightreading);
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().left,
  robot_simulator->get_motion_handler()->get_velocity().left)
  << "Fail: velocity of the left wheel not correct when only 1 light";
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().right,
  robot_simulator->get_motion_handler()->get_velocity().right)
  << "Fail: velocity of the right wheel not correct when only 1 light";
  // Imagine that there are 4 lights at (100,120), (300,300),(450,500),(600,610)
  lightsensorleft->set_sensor_reading(0);
  lightsensorright->set_sensor_reading(0);
  robot->get_left_light_sensor()->set_sensor_reading(0.0);
  robot->get_right_light_sensor()->set_sensor_reading(0.0);
  robot->get_motion_handler()->set_velocity(5, 5);
  robot_simulator->get_motion_handler()->set_velocity(5, 5);
  light1->set_position(100,120);
  robot->RobotDecideMotion(light1->get_type(), light1);
  lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                               get_sensor_position());
  lightsensorleft->CalculateSensorReading({100,120});
  lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                               get_sensor_position());
  lightsensorright->CalculateSensorReading({100,120});
  leftreading = lightsensorleft->get_sensor_reading();
  rightreading = lightsensorright->get_sensor_reading();
  VelocityWheel(1, leftreading, rightreading);
  csci3081::Light *light2 = new csci3081::Light();
  light2->set_position(300,300);
  robot->RobotDecideMotion(light2->get_type(), light2);
  lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                               get_sensor_position());
  lightsensorleft->CalculateSensorReading({300,300});
  lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                               get_sensor_position());
  lightsensorright->CalculateSensorReading({300,300});
  leftreading = lightsensorleft->get_sensor_reading();
  rightreading = lightsensorright->get_sensor_reading();
  VelocityWheel(1, leftreading, rightreading);
  csci3081::Light *light3 = new csci3081::Light();
  light3->set_position(450,500);
  robot->RobotDecideMotion(light3->get_type(), light3);
  lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                               get_sensor_position());
  lightsensorleft->CalculateSensorReading({450,500});
  lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                               get_sensor_position());
  lightsensorright->CalculateSensorReading({450,500});
  leftreading = lightsensorleft->get_sensor_reading();
  rightreading = lightsensorright->get_sensor_reading();
  VelocityWheel(1, leftreading, rightreading);
  csci3081::Light *light4 = new csci3081::Light();
  light4->set_position(600,610);
  robot->RobotDecideMotion(light4->get_type(), light4);
  lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                               get_sensor_position());
  lightsensorleft->CalculateSensorReading({600,610});
  lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                               get_sensor_position());
  lightsensorright->CalculateSensorReading({600,610});
  leftreading = lightsensorleft->get_sensor_reading();
  rightreading = lightsensorright->get_sensor_reading();
  VelocityWheel(1, leftreading, rightreading);
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().
  left,robot_simulator->get_motion_handler()->get_velocity().left)
  << "Fail: velocity of the left wheel not correct when multiple lights";
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().right,
  robot_simulator->get_motion_handler()->get_velocity().right)
  << "Fail: velocity of the right wheel not correct when multiple lights";
  // Testing for maximum velocity of the robot when it is fearing light
  // Happens when robot is close to the light
  robot->get_motion_handler()->set_velocity(0,0);
  robot->get_left_light_sensor()->set_sensor_reading(0.0);
  robot->get_right_light_sensor()->set_sensor_reading(0.0);
  robot->set_position(600,600);
  light1->set_position(620,620);
  robot->RobotDecideMotion(light1->get_type(), light1);
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().left,10)
  << "Fail: velocity of the left wheel should be maximum";
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().right,10)
  << "Fail: velocity of the right wheel should be maximum";
  // Testing for minimum velocity of the robot when it is fearing light
  // Happens when robot is far away from the light
  lightsensorleft->set_sensor_reading(0);
  lightsensorright->set_sensor_reading(0);
  robot->get_left_light_sensor()->set_sensor_reading(0.0);
  robot->get_right_light_sensor()->set_sensor_reading(0.0);
  robot->get_motion_handler()->set_velocity(5, 5);
  robot_simulator->get_motion_handler()->set_velocity(5, 5);
  robot->set_position(100,100);
  light1->set_position(1000,1000);
  robot->RobotDecideMotion(light1->get_type(), light1);
  lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                               get_sensor_position());
  lightsensorleft->CalculateSensorReading({1000,1000});
  lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                               get_sensor_position());
  lightsensorright->CalculateSensorReading({1000,1000});
  leftreading = lightsensorleft->get_sensor_reading();
  rightreading = lightsensorright->get_sensor_reading();
  VelocityWheel(1, leftreading, rightreading);
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().
  left,robot_simulator->get_motion_handler()->get_velocity().left)
  << "Fail: velocity of the left wheel should be very close to 0";
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().
  right,robot_simulator->get_motion_handler()->get_velocity().right)
  << "Fail: velocity of the right wheel should be very close to 0";
}
// before each set of tests, setting the robot velocity to 5.
// leftreading is the left sensor reading
// rightreading is the right sensor reading
// This method tests Robot's exploratory behavior towards light.
TEST_F(UpdateVelocityTest, NegativeCrossConnection) {
  // Imagine that there is one light at (350,350)
  robot->set_behavior_flag(0);
  lightsensorleft->set_sensor_reading(0);
  lightsensorright->set_sensor_reading(0);
  robot->get_left_light_sensor()->set_sensor_reading(0.0);
  robot->get_right_light_sensor()->set_sensor_reading(0.0);
  robot->get_motion_handler()->set_velocity(5, 5);
  robot_simulator->get_motion_handler()->set_velocity(5, 5);
  csci3081::Light *light1 = new csci3081::Light();
  light1->set_position(350,350);
  robot->RobotDecideMotion(light1->get_type(), light1);
  lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                              get_sensor_position());
  lightsensorleft->CalculateSensorReading({350,350});
  lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                              get_sensor_position());
  lightsensorright->CalculateSensorReading({350,350});
  double leftreading = lightsensorleft->get_sensor_reading();
  double rightreading = lightsensorright->get_sensor_reading();
  VelocityWheel(2, leftreading, rightreading);
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().left,
  robot_simulator->get_motion_handler()->get_velocity().left)
  << "Fail: velocity of the left wheel not correct when only 1 light";
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().right,
  robot_simulator->get_motion_handler()->get_velocity().right)
  << "Fail: velocity of the right wheel not correct when only 1 light";
  // Imagine there are 4 lights at (235,235), (400, 300), (110,110) (780,680)
  lightsensorleft->set_sensor_reading(0);
  lightsensorright->set_sensor_reading(0);
  robot->get_left_light_sensor()->set_sensor_reading(0.0);
  robot->get_right_light_sensor()->set_sensor_reading(0.0);
  robot->get_motion_handler()->set_velocity(5, 5);
  robot_simulator->get_motion_handler()->set_velocity(5, 5);
  light1->set_position(235,235);
  robot->RobotDecideMotion(light1->get_type(), light1);
  lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                               get_sensor_position());
  lightsensorleft->CalculateSensorReading({235,235});
  lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                               get_sensor_position());
  lightsensorright->CalculateSensorReading({235,235});
  leftreading = lightsensorleft->get_sensor_reading();
  rightreading = lightsensorright->get_sensor_reading();
  VelocityWheel(2, leftreading, rightreading);
  csci3081::Light *light2 = new csci3081::Light();
  light2->set_position(400,300);
  robot->RobotDecideMotion(light2->get_type(), light2);
  lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                               get_sensor_position());
  lightsensorleft->CalculateSensorReading({400,300});
  lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                               get_sensor_position());
  lightsensorright->CalculateSensorReading({400,300});
  leftreading = lightsensorleft->get_sensor_reading();
  rightreading = lightsensorright->get_sensor_reading();
  VelocityWheel(2, leftreading, rightreading);
  csci3081::Light *light3 = new csci3081::Light();
  light3->set_position(110,110);
  robot->RobotDecideMotion(light3->get_type(), light3);
  lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                               get_sensor_position());
  lightsensorleft->CalculateSensorReading({110,110});
  lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                               get_sensor_position());
  lightsensorright->CalculateSensorReading({110,110});
  leftreading = lightsensorleft->get_sensor_reading();
  rightreading = lightsensorright->get_sensor_reading();
  VelocityWheel(2, leftreading, rightreading);
  csci3081::Light *light4 = new csci3081::Light();
  light4->set_position(780,680);
  robot->RobotDecideMotion(light4->get_type(), light4);
  lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                               get_sensor_position());
  lightsensorleft->CalculateSensorReading({780,680});
  lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                               get_sensor_position());
  lightsensorright->CalculateSensorReading({780,680});
  leftreading = lightsensorleft->get_sensor_reading();
  rightreading = lightsensorright->get_sensor_reading();
  VelocityWheel(2, leftreading, rightreading);
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().
  left,robot_simulator->get_motion_handler()->get_velocity().left)
  << "Fail: velocity of the left wheel not correct when multiple lights";
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().right,
  robot_simulator->get_motion_handler()->get_velocity().right)
  << "Fail: velocity of the left wheel not correct when multiple lights";
  // Testing for maximum velocity of the robot when it is exploring light
  // Happens when robot is far away from light
  robot->get_motion_handler()->set_velocity(0,0);
  robot->get_left_light_sensor()->set_sensor_reading(0.0);
  robot->get_right_light_sensor()->set_sensor_reading(0.0);
  robot->set_position(100,100);
  light1->set_position(700,700);
  robot->RobotDecideMotion(light1->get_type(), light1);
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().left,10)
  << "Fail: velocity of the left wheel should be maximum";
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().right,10)
  << "Fail: velocity of the right wheel should be maximum";
  // Testing for minimum velocity of the robot when it is exploring light
  // Happens when light and robot are very close to each other
  lightsensorleft->set_sensor_reading(0);
  lightsensorright->set_sensor_reading(0);
  robot->get_left_light_sensor()->set_sensor_reading(0.0);
  robot->get_right_light_sensor()->set_sensor_reading(0.0);
  robot->get_motion_handler()->set_velocity(9, 9);
  robot_simulator->get_motion_handler()->set_velocity(9, 9);
  robot->set_position(100,100);
  light1->set_position(110,110);
  robot->RobotDecideMotion(light1->get_type(), light1);
  lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                               get_sensor_position());
  lightsensorleft->CalculateSensorReading({110,110});
  lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                               get_sensor_position());
  lightsensorright->CalculateSensorReading({110,110});
  leftreading = lightsensorleft->get_sensor_reading();
  rightreading = lightsensorright->get_sensor_reading();
  VelocityWheel(2, leftreading, rightreading);
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().
  left,robot_simulator->get_motion_handler()->get_velocity().left)
  << "Fail: velocity of the left wheel should be very close to 0";
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().
  right,robot_simulator->get_motion_handler()->get_velocity().right)
  << "Fail: velocity of the right wheel should be very close to 0";
}

// Testing for robot not reacting to light when it is really hungry
TEST_F(UpdateVelocityTest, BehavioralChecks) {
  // First checking for the velocity when robot is not "really hungry".
  // When the robot is really hungry, it's motion should be getting impacted by light.
  robot->set_behavior_flag(1);
  lightsensorleft->set_sensor_reading(0);
  lightsensorright->set_sensor_reading(0);
  robot->get_left_light_sensor()->set_sensor_reading(0.0);
  robot->get_right_light_sensor()->set_sensor_reading(0.0);
  robot->get_motion_handler()->set_velocity(5, 5);
  robot_simulator->get_motion_handler()->set_velocity(5,5);
  csci3081::Light *light1 = new csci3081::Light();
  light1->set_position(700,700);
  (robot->set_really_hungry(false)); // setting the reall hungry flag of robot to false
  // fear and not really hungry
  if (robot->get_behavior_flag()%2==1) {
    robot->RobotDecideMotion(light1->get_type(), light1);
    lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                               get_sensor_position());
    lightsensorleft->CalculateSensorReading({700,700});
    lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                               get_sensor_position());
    lightsensorright->CalculateSensorReading({700,700});
    double leftreading = lightsensorleft->get_sensor_reading();
    double rightreading = lightsensorright->get_sensor_reading();
    VelocityWheel(1, leftreading, rightreading);
  }
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().
  left,robot_simulator->get_motion_handler()->get_velocity().left)
  << "Fail: velocity of the left wheel should be changing";
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().right,
  robot_simulator->get_motion_handler()->get_velocity().right)
  << "Fail: velocity of the right wheel be changing";
  robot->set_behavior_flag(0);
  lightsensorleft->set_sensor_reading(0);
  lightsensorright->set_sensor_reading(0);
  robot->get_left_light_sensor()->set_sensor_reading(0.0);
  robot->get_right_light_sensor()->set_sensor_reading(0.0);
  robot->get_motion_handler()->set_velocity(5, 5);
  robot_simulator->get_motion_handler()->set_velocity(5,5);
  // Explore and not really hungry
  if (robot->get_behavior_flag()%2==0) {
    robot->RobotDecideMotion(light1->get_type(), light1);
    lightsensorleft->set_sensor_position(robot->get_left_light_sensor()->
                                               get_sensor_position());
    lightsensorleft->CalculateSensorReading({700,700});
    lightsensorright->set_sensor_position(robot->get_right_light_sensor()->
                                               get_sensor_position());
    lightsensorright->CalculateSensorReading({700,700});
    double leftreading = lightsensorleft->get_sensor_reading();
    double rightreading = lightsensorright->get_sensor_reading();
    VelocityWheel(2, leftreading, rightreading);
  }
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().
  left,robot_simulator->get_motion_handler()->get_velocity().left)
  << "Fail: velocity of the left wheel should be changing";
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().right,
  robot_simulator->get_motion_handler()->get_velocity().right)
  << "Fail: velocity of the right wheel should be changing";
  // Second, checking for the velocity when the robot is really hungry.
  // In this case, the light sensors won't be impacting the motion of the robots
  robot->get_left_light_sensor()->set_sensor_reading(0.0);
  robot->get_right_light_sensor()->set_sensor_reading(0.0);
  robot->get_motion_handler()->set_velocity(5, 5);
  (robot->set_really_hungry(true));// forcefully making the robot really hungry
  // Fear and not really hungry
  if (robot->get_behavior_flag()%2==1) {
    robot->RobotDecideMotion(light1->get_type(), light1);
  }
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().left, 5)
  << "Fail: velocity of the wheel should not change";
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().right, 5)
  << "Fail: velocity of the wheel should not change";
  robot->set_behavior_flag(0);
  // explore and really hungry
  if (robot->get_behavior_flag()%2==0) {
    robot->RobotDecideMotion(light1->get_type(), light1);
  }
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().left, 5)
  << "Fail: velocity of the wheel should not change";
  EXPECT_DOUBLE_EQ(robot->get_motion_handler()->get_velocity().right, 5)
  << "Fail: velocity of the wheel should not change";
}

#endif /* MOTIONHANDLER_TESTS */
