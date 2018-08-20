
/**
 * @file robot.cc
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <cmath>
#include "src/robot.h"
#include "src/params.h"
#include "src/food.h"
#include "src/pose.h"
#include "src/arena_entity.h"
#include "src/motion_handler_robot.h"
#include "src/motion_handler_aggression.h"
#include "src/motion_handler_exploratory.h"
#include "src/motion_handler_fear.h"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
Robot::Robot(int behavior) :
    motion_handler_(),
    motion_behavior_(this),
    lives_(9),
    time_counter_(0),
    collision_tracker_(false),
    right_light_sensor_(PoseRightSensor()),
    left_light_sensor_(PoseLeftSensor()),
    right_food_sensor_(PoseRightSensor()),
    left_food_sensor_(PoseLeftSensor()),
    behavior_light_flag_(behavior),
    hunger_tracker_(false),
    collision_timer_(0),
    sensitivity_to_light_(0.0f),
    food_on_off_(0),
    really_hungry_(false) {
  set_type(kRobot);
  set_color(ROBOT_COLOR);
  set_pose(ROBOT_INIT_POS);
  set_heading(0);
  set_radius(ROBOT_RADIUS);
  motion_handler_ = new MotionHandlerRobot(this);
  motion_handler_->set_velocity(5, 5);
}
/*******************************************************************************
 * Member Functions
 ******************************************************************************/
/* Updating robot's velocity and position at each timestep. */
void Robot::TimestepUpdate(unsigned int dt) {
  motion_behavior_.UpdatePose(dt, motion_handler_->get_velocity());
  left_light_sensor_.set_sensor_reading(0.0);
  right_light_sensor_.set_sensor_reading(0.0);
  left_food_sensor_.set_sensor_reading(0.0);
  right_food_sensor_.set_sensor_reading(0.0);
  time_counter_ += 1;
  RobotStateUpdate();
} /* TimestepUpdate() */

void Robot::RobotStateUpdate() {
// Checking if the robot is currently in a state of arcing or not.
  if (collision_tracker_) {
    collision_timer_ += 1;
    motion_handler_->set_velocity(6, 6);
    ArcMovement();
  }
// Keeping track of the time for which the robot should be arcing.
  if (collision_timer_ == 25) {
    collision_timer_ = 0;
    collision_tracker_ = false;
  }
// approximately 21 timestep update corresponds to 1 second.
// therefore 30 seconds will be equivalent to 620 timestep updates
  if ( time_counter_ == 620 ) {
    hunger_tracker_ = true;
  }
  if (food_on_off_ == 0) {  // the robot shouldn't get hungry if food is off
    hunger_tracker_ = false;
  }
  /* flickering the robot lights to show robot is hungry.
  * Yellow color denotes that the robot is really hungry.
  */
  if (hunger_tracker_) {
    if ( time_counter_ < 20*120 ) {
      if (time_counter_ % 2 == 1) {
        set_color(CHANGED_ROBOT_COLOR1);
      } else {
        set_color(CHANGED_ROBOT_COLOR2);
      }
    } else {
      set_color(CHANGED_ROBOT_COLOR3);
    }
  }
  sensor_touch_->Reset();
}

void Robot::Reset() {
  set_pose(SetPoseRandomly());
  motion_handler_->set_max_speed(ROBOT_MAX_SPEED);
  motion_handler_->set_max_angle(ROBOT_MAX_ANGLE);
  motion_handler_->set_velocity(5, 5);
  set_radius(SetRadiusRandomlyRobot());
  sensor_touch_->Reset();
  set_color(ROBOT_COLOR);
  right_food_sensor_.set_sensor_position((PoseRightSensor()));
  left_food_sensor_.set_sensor_position((PoseLeftSensor()));
  right_light_sensor_.set_sensor_position((PoseRightSensor()));
  left_light_sensor_.set_sensor_position((PoseLeftSensor()));
  right_light_sensor_.set_sensor_reading(0.0);
  right_food_sensor_.set_sensor_reading(0.0);
  left_light_sensor_.set_sensor_reading(0.0);
  left_food_sensor_.set_sensor_reading(0.0);
  // lives_ = 9;
  time_counter_ = 0;
  collision_tracker_ = false;
  hunger_tracker_ = false;
  collision_timer_ = 0;
} /* Reset() */


// Handling collisions of robots with other entities and walls
void Robot::HandleCollision(EntityType object_type, ArenaEntity * object) {
  switch (object_type) {
    case (kTopWall) : sensor_touch_->HandleCollision(object_type, object);
    collision_tracker_ = true;
    collision_timer_ = 0;
    (this)->RelativeChangeHeading(170);
    break;
    case (kBottomWall) : sensor_touch_->HandleCollision(object_type, object);
    collision_tracker_ = true;
    collision_timer_ = 0;
    (this)->RelativeChangeHeading(170);
    break;
    case (kLeftWall) : sensor_touch_->HandleCollision(object_type, object);
    collision_tracker_ = true;
    collision_timer_ = 0;
    (this)->RelativeChangeHeading(170);
    break;
    case (kRightWall) : sensor_touch_->HandleCollision(object_type, object);
    collision_tracker_ = true;
    collision_timer_ = 0;
    (this)->RelativeChangeHeading(170);
    break;
    case (kRobot) : sensor_touch_->HandleCollision(object_type, object);
    collision_tracker_ = true;
    collision_timer_ = 0;
    (this)->RelativeChangeHeading(170);
    break;
    default : break;
  }
}

void Robot::IncreaseSpeed() {
  motion_handler_->IncreaseSpeed();
}

void Robot::DecreaseSpeed() {
  motion_handler_->DecreaseSpeed();
}

void Robot::TurnRight() {
  motion_handler_->TurnRight();
}

void Robot::TurnLeft() {
  motion_handler_->TurnLeft();
}

// Function that gives robot it's arc movement.
void Robot::ArcMovement() {
  motion_handler_->IncreaseSpeed();
  left_light_sensor_.set_sensor_position(PoseLeftSensor());
  right_light_sensor_.set_sensor_position(PoseRightSensor());
  left_food_sensor_.set_sensor_position(PoseLeftSensor());
  right_food_sensor_.set_sensor_position(PoseRightSensor());
}

// For the entity, based on robot's behavior and position, the robot determines
// if it's sensors have been triggered or not and in case it has been triggered,
// robot decides the plan of action for that entity and implements a particular
// motion.
void Robot::RobotDecideMotion(EntityType object_type,
                                               ArenaEntity * ent) {
  right_light_sensor_.set_sensor_position(PoseRightSensor());
  left_food_sensor_.set_sensor_position(PoseLeftSensor());
  right_food_sensor_.set_sensor_position(PoseRightSensor());
  left_light_sensor_.set_sensor_position(PoseLeftSensor());
  switch (object_type) {
    case (kLight) : if (!really_hungry_) {  // Robot reacts to light only
    // till the point when it is not really hungry.
    left_light_sensor_.CalculateSensorReading(ent->get_pose());
    right_light_sensor_.CalculateSensorReading(ent->get_pose());
    double l = left_light_sensor_.get_sensor_reading();
    double r = right_light_sensor_.get_sensor_reading();
    if (behavior_light_flag_ % 2 == 1) {  // Checking if the robot fears light
      // or explores light and then taking the appropriate plan of action.
      MotionHandlerFear *fear
          = new MotionHandlerFear(this);
      WheelVelocity v = fear->UpdateVelocity(l, r,
                         motion_handler_->get_velocity());
      motion_handler_->set_velocity(v);
    } else {
      MotionHandlerExploratory *explore
          = new MotionHandlerExploratory(this);
      WheelVelocity v = explore->UpdateVelocity(l, r,
                         motion_handler_->get_velocity());
      motion_handler_->set_velocity(v);
    }
    }
    break;
    case (kFood) : if (IsFoodConsumed(ent)) {  // checking if the food has been
    // consumed.
    hunger_tracker_ = false;
    time_counter_ = 0;
    set_color(ROBOT_COLOR);
    motion_handler_->set_velocity(5, 5);
  } else if (hunger_tracker_) {  // checking if the robot needs to be aggressive
    // or not.
    left_food_sensor_.CalculateSensorReading(ent->get_pose());
    right_food_sensor_.CalculateSensorReading(ent->get_pose());
    double l = left_food_sensor_.get_sensor_reading();
    double r = right_food_sensor_.get_sensor_reading();
    MotionHandlerAggression *aggression
        = new MotionHandlerAggression(this);
    WheelVelocity v = aggression->UpdateVelocity(l, r,
                       motion_handler_->get_velocity());
    motion_handler_->set_velocity(v);
    }
    break;
    default : break;
  }
  // To ensure proper movement of the robot when it is either hungry or arcing.
  if (hunger_tracker_ || collision_tracker_) {
    motion_handler_->set_velocity(
      motion_handler_->clamp_vel
          (motion_handler_->get_velocity().left+5.0),
      motion_handler_->clamp_vel
          (motion_handler_->get_velocity().left+5.0));
    }
}

/* Robot is very hungry if it has not consumed food for 2 mins. Returns true if
* the robot is very hungry else false.
*/
bool Robot::RobotReallyHungry() {
  // 21 timestep update corresponds to approximately 1 second. So in 2 mins,
  // number of timestep update will be 21*120 second.
  if (time_counter_ >= 20*120 && hunger_tracker_) {
    motion_handler_->set_velocity(7, 7);
    really_hungry_ = true;
  } else {
    really_hungry_ = false;
  }
  return really_hungry_;
}
/* Robot is starving if it has not consumed food for 2.5 mins. Returns true
* if robot is starving else false.
*/
bool Robot::RobotStarving() {
  // 21 timestep update corresponds to approximately 1 second. So in 2 mins,
  // number of timestep update will be 21*150 second.
  return time_counter_ >= 20*150 && hunger_tracker_;
}

/* Calculates the distance between food and robot to check if robot
* is going to consume that food entity or not.
*/
bool Robot::IsFoodConsumed(ArenaEntity * const other_e) {
    double delta_x = other_e->get_pose().x - (this)->get_pose().x;
    double delta_y = other_e->get_pose().y - (this)->get_pose().y;
    double distance_between_ = sqrt(delta_x*delta_x + delta_y*delta_y);
    return
    (distance_between_ <= ((this)->get_radius() +
                                            other_e->get_radius()+5.0));
}

void Robot::set_sensitivity_to_light(float lsensor_base) {
  left_light_sensor_.set_base_value(lsensor_base);
  right_light_sensor_.set_base_value(lsensor_base);
}

NAMESPACE_END(csci3081);
