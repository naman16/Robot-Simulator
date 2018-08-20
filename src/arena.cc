/**
 * @file arena.cc
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <iostream>
#include "src/food.h"
#include "src/arena.h"
#include "src/arena_params.h"
#include "src/light.h"
#include "src/sensor.h"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
Arena::Arena(const struct arena_params *const params)
    : x_dim_(params->x_dim),
      y_dim_(params->y_dim),
      factory_(new EntityFactory),
      robot_(),
      entities_(),
      mobile_entities_(),
      game_status_(PLAYING),
      robot_count_(5),
      light_count_(0),
      food_count_(0) {
  // AddRobot(kRobot, robot_count_);
  // AddFood(kFood, food_count_);
  // AddLight(kLight, light_count_);
}

Arena::~Arena() {
  for (auto ent : entities_) {
    delete ent;
  } /* for(ent..) */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

// Function for adding robots to the arena.
void Arena::AddRobot(EntityType type, int quantity) {
  robot_.clear();
  entities_.clear();
  mobile_entities_.clear();
  factory_ -> Reset();
  for (int i = 0; i < quantity; i++) {
    Robot * robot_temp_ = dynamic_cast<Robot *>(factory_->CreateEntity(type));
    robot_.push_back(robot_temp_);
    entities_.push_back(robot_temp_);
    mobile_entities_.push_back(robot_temp_);
  }
}

// Function for adding lights to the arena
void Arena::AddLight(EntityType type, int quantity) {
  for (int i = 0; i < quantity; i++) {
    Light *obs_ = dynamic_cast<Light*>(factory_->CreateEntity(type));
    entities_.push_back(obs_);
    mobile_entities_.push_back(obs_);
  }
}
// Function for adding foods to the arena
void Arena::AddFood(EntityType type, int quantity) {
  for (int i = 0; i < quantity; i++) {
    entities_.push_back(factory_->CreateEntity(type));
  }
}

// Function for reseting the arena in order to prepare for a New Game.
void Arena::Reset() {
  set_game_status(PLAYING);
  for (auto ent : entities_) {
    ent->Reset();
  } /* for(ent..) */
} /* reset() */

// The primary driver of simulation movement. Called from the Controller
// but originated from the graphics viewer.
void Arena::AdvanceTime(double dt) {
  if (!(dt > 0)) {
    return;
  }
  for (size_t i = 0; i < 1; ++i) {
    UpdateEntitiesTimestep();
  } /* for(i..) */
} /* AdvanceTime() */

void Arena::UpdateEntitiesTimestep() {
  /*
   * First, update the position of all entities, according to their current
   * velocities.
   * @TODO: Should this be just the mobile entities ??
   */
  for (auto ent : mobile_entities_) {
    ent->TimestepUpdate(1);
  }
  // Checking if any of the robots in the arena are starving or not because
  // if they are, then the simulation should be over.
  for (auto& robot : robot_) {
    if (robot->RobotStarving()) {
      set_game_status(LOST);
    }
  }
   /* Determine if any mobile entity is colliding with wall.
   * Adjust the position accordingly so it doesn't overlap.
   */
  for (auto &ent1 : mobile_entities_) {
     EntityType wall = GetCollisionWall(ent1);
     if (kUndefined != wall) {
       AdjustWallOverlap(ent1, wall);
       if (ent1->get_type() == kRobot) {
         dynamic_cast <Robot*> (ent1)-> HandleCollision(wall);
       } else if (ent1->get_type() == kLight) {
         dynamic_cast <Light*> (ent1)-> HandleCollision(wall);
       }
     }
    /* Determine if that mobile entity is colliding with any other entity.
    * Adjust the position accordingly so they don't overlap.
    */
    for (auto &ent2 : entities_) {
      if (ent2 == ent1) { continue; }
      if (IsColliding(ent1, ent2)) {
        AdjustEntityOverlap(ent1, ent2);
        if (ent1->get_type() == kRobot) {
          dynamic_cast <Robot*> (ent1)-> HandleCollision
          (ent2->get_type(), ent2);
        } else {
          dynamic_cast <Light*> (ent1)-> HandleCollision
          (ent2->get_type(), ent2);
        }
      }
    }
  }
  /* Checking if any of the light or food sensors of the robot
  * have been triggered or not by sending the entity to robot class
  * and letting robot decide for itself by calculating its sensor reading
  * based on the position of the entity. All this is happening in the Robot
  * class since robot is the observer.
  */
  for (auto& robot : robot_) {
    for (auto& ent : entities_) {
      robot->RobotDecideMotion(ent->get_type(), ent);
    }
  }
}  // UpdateEntitiesTimestep()

/* Determine if the entity is colliding with a wall.
 * Always returns an entity type. If not collision, returns kUndefined.
 */
EntityType Arena::GetCollisionWall(ArenaMobileEntity *const ent) {
  if (ent->get_pose().x + ent->get_radius() >= x_dim_) {
    return kRightWall;  // at x = x_dim_
  } else if (ent->get_pose().x - ent->get_radius() <= 0) {
    return kLeftWall;  // at x = 0
  } else if (ent->get_pose().y + ent->get_radius() >= y_dim_) {
    return kBottomWall;  // at y = y_dim
  } else if (ent->get_pose().y - ent->get_radius() <= 0) {
    return kTopWall;  // at y = 0
  } else {
    return kUndefined;
  }
} /* GetCollisionWall() */

/* The entity type indicates which wall the entity is colliding with.
* This determines which way to move the entity to set it slightly off the wall.
*/
void Arena::AdjustWallOverlap(ArenaMobileEntity *const ent, EntityType object) {
  Pose entity_pos = ent->get_pose();
  switch (object) {
    case (kRightWall):  // at x = x_dim_
    ent->set_position(x_dim_-(ent->get_radius()+5), entity_pos.y);
    break;
    case (kLeftWall):  // at x = 0
    ent->set_position(ent->get_radius()+5, entity_pos.y);
    break;
    case (kTopWall):  // at y = 0
    ent->set_position(entity_pos.x, ent->get_radius()+5);
    break;
    case (kBottomWall):  // at y = y_dim_
    ent->set_position(entity_pos.x, y_dim_-(ent->get_radius()+5));
    break;
    default:
    {}
  }
}

/* Calculates the distance between the center points to determine overlap */
bool Arena::IsColliding(
  ArenaMobileEntity * const mobile_e,
  ArenaEntity * const other_e) {
    double delta_x = other_e->get_pose().x - mobile_e->get_pose().x;
    double delta_y = other_e->get_pose().y - mobile_e->get_pose().y;
    double distance_between = sqrt(delta_x*delta_x + delta_y*delta_y);
    return
    (distance_between <= (mobile_e->get_radius() + other_e->get_radius()));
}


/* This is called when it is known that the two entities overlap.
* We determine by how much they overlap then move the mobile entity to
* the edge of the other
*/
/* @TODO: Add functionality to Pose to determine the distance distance_between two instances (e.g. overload operator -)
*/
/* @BUG: The robot will pass through the home food on occasion. The problem
 * is likely due to the adjustment being in the wrong direction. This could
 * be because the cos/sin generate the wrong sign of the distance_to_move
 * when the collision is in a specific quadrant relative to the center of the
 * colliding entities..
 */
void Arena::AdjustEntityOverlap(ArenaMobileEntity * const mobile_e,
  ArenaEntity *const other_e) {
    double delta_x = mobile_e->get_pose().x - other_e->get_pose().x;
    double delta_y = mobile_e->get_pose().y - other_e->get_pose().y;
    double distance_between = sqrt(delta_x*delta_x + delta_y*delta_y);
    double distance_to_move =
      mobile_e->get_radius() + other_e->get_radius() - distance_between;
    double angle = atan2(delta_y, delta_x);
    mobile_e->set_position(
     mobile_e->get_pose().x+cos(angle)*distance_to_move,
     mobile_e->get_pose().y+cos(angle)*distance_to_move);
}


// Accept communication from the controller. Dispatching as appropriate.
/** @TODO: Call the appropriate Robot functions to implement user input
  * for controlling the robot.
  */
void Arena::AcceptCommand(Communication com) {
  switch (com) {
    case(kIncreaseSpeed):  // robot_ -> IncreaseSpeed();
    break;
    case(kDecreaseSpeed):  // robot_ -> DecreaseSpeed();
    break;
    case(kTurnLeft):  // robot_ -> TurnLeft();
    break;
    case(kTurnRight):  // robot_ -> TurnRight();
    break;
    case(kPlay):
    break;
    case(kPause):
    break;
    case(kReset): Reset();
    break;
    case(kNone): break;
    default: break;
  }
} /* AcceptCommand */

// 1 denotes fear behavior, 0 denotes exploratory behavior.
// 1 denotes food is on, 0 denotes food is off
// This function ensures the desired number of robots fear light and also
// makes sure that the robot knows if food is on or off.
void Arena::set_behavior_sensitivity_robot(int fcount, float lsensor_base,
                                                      int fonoff) {
  for (auto& robot : robot_) {
    if (fcount > 0) {
      robot->set_behavior_flag(1);
      fcount--;
    } else {
      robot->set_behavior_flag(0);
    }
    robot-> set_sensitivity_to_light(lsensor_base);
    robot-> set_food_on_off(fonoff);
  }
}



NAMESPACE_END(csci3081);
