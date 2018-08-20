/**
 * @file light.cc
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "src/light.h"
#include "src/params.h"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
Light::Light() :
    motion_handler_(this),
    motion_behavior_(this) {
  set_color(LIGHT_COLOR);
  set_pose(LIGHT_POSITION);
  set_radius(LIGHT_RADIUS);
  set_type(kLight);
  motion_handler_.set_velocity(LIGHT_SPEED, LIGHT_SPEED);
}

void Light::Reset() {
  set_pose(SetPoseRandomly());
  set_type(kLight);
  set_color(LIGHT_COLOR);
  set_radius(LIGHT_RADIUS);
  sensor_touch_->Reset();
  motion_handler_.set_velocity(LIGHT_SPEED, LIGHT_SPEED);
}/* Reset */


void Light::TimestepUpdate(unsigned int dt) {
  motion_behavior_.UpdatePose(dt, motion_handler_.get_velocity());
  // Reset Sensor for next cycle
  sensor_touch_->Reset();
} /* TimestepUpdate() */


// Function to deal with the different cases when
// light undergoes collision
void Light::HandleCollision(EntityType object_type, ArenaEntity * object) {
  switch (object_type) {
    case (kLight) : sensor_touch_->HandleCollision(object_type, object);
    ArcMovement();
    motion_handler_.set_velocity(LIGHT_SPEED, LIGHT_SPEED);
    break;
    case (kTopWall) : sensor_touch_->HandleCollision(object_type, object);
    ArcMovement();
    motion_handler_.set_velocity(LIGHT_SPEED, LIGHT_SPEED);
    break;
    case (kBottomWall) : sensor_touch_->HandleCollision(object_type, object);
    ArcMovement();
    motion_handler_.set_velocity(LIGHT_SPEED, LIGHT_SPEED);
    break;
    case (kLeftWall) : sensor_touch_->HandleCollision(object_type, object);
    ArcMovement();
    motion_handler_.set_velocity(LIGHT_SPEED, LIGHT_SPEED);
    break;
    case (kRightWall) : sensor_touch_->HandleCollision(object_type, object);
    ArcMovement();
    motion_handler_.set_velocity(LIGHT_SPEED, LIGHT_SPEED);
    break;
    // case (kFood) : ArcMovement();
    // motion_handler_.set_velocity(LIGHT_SPEED, LIGHT_SPEED);
    // case(kRobot) : ArcMovement();
    // motion_handler_.set_velocity(LIGHT_SPEED, LIGHT_SPEED);
    // break;
    default : break;
  }
}

// Function that gives lights the arc movement when lights collide
void Light::ArcMovement() {
  /*
   * Changing the headingangle of the light by 180 and then
   * running a while loop 10 times in which the angle is made
   * negative and changing the angle by 10 in each iteration.
   * Furthermore, in every iteration of the while loop,
   * decreasing the left wheel velocity of the light by 1 and increasing
   * the right wheel velocity by 1 and also updating the position according
   * to the new velocity and angle.
   */
  double dt = 10;
  int angle = 180;
  while (dt >= 0) {
    // static_cast<double>  (random() %
    // (LIGHT_MAX_ANGLE + 1 - LIGHT_MIN_ANGLE) + LIGHT_MIN_ANGLE);
    (this)->RelativeChangeHeading(angle);;
    motion_handler_.set_velocity(motion_handler_.get_velocity().left-1,
    motion_handler_.get_velocity().right+1);
    motion_behavior_.UpdatePose(dt, motion_handler_.get_velocity());
    // adjusting angle so that it is moving backwards in an arc
    // doing the adjustment foodd on the sign of the angle so that it
    // doesn't get stuck
    if (angle > 0) {
      angle = angle + 10;
    } else {
      angle = -angle + 10;
    }
    dt = dt - 1;
  }
}

NAMESPACE_END(csci3081);
