/**
 * @file entity_factory.cc
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <ctime>
#include <iostream>

#include "src/common.h"
#include "src/entity_factory.h"
#include "src/entity_type.h"
#include "src/params.h"
#include "src/pose.h"
#include "src/rgb_color.h"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

EntityFactory::EntityFactory() {
  srand(time(nullptr));
}

void EntityFactory::Reset() {
  entity_count_ = 0;
  robot_count_ = 0;
  light_count_ = 0;
  food_count_ = 0;
  robot_light_behavior_flag_ = 1;
}
ArenaEntity* EntityFactory::CreateEntity(EntityType etype) {
  switch (etype) {
    case (kRobot):
      return CreateRobot();
      break;
    case (kLight):
      return CreateLight();
      break;
    case (kFood):
      return CreateFood();
      break;
    default:
      std::cout << "FATAL: Bad entity type on creation\n";
      assert(false);
  }
  return nullptr;
}

Robot* EntityFactory::CreateRobot() {
  auto* robot = new Robot(robot_light_behavior_flag_);
  robot->set_type(kRobot);
  robot->set_color(ROBOT_COLOR);
  robot->set_pose(ROBOT_INIT_POS);
  robot->set_radius(SetRadiusRandomlyRobot());
  robot->set_pose(SetPoseRandomly());
  ++entity_count_;
  ++robot_count_;
  ++robot_light_behavior_flag_;
  robot->set_id(robot_count_);
  return robot;
}

Light* EntityFactory::CreateLight() {
  auto* light = new Light;
  light->set_type(kLight);
  light->set_color(LIGHT_COLOR);
  light->set_pose(SetPoseRandomly());
  light->set_radius(LIGHT_RADIUS);
  ++entity_count_;
  ++light_count_;
  light->set_id(light_count_);
  return light;
}

Food* EntityFactory::CreateFood() {
  auto* food = new Food;
  food->set_type(kFood);
  food->set_color(FOOD_COLOR);
  food->set_pose(SetPoseRandomly());
  food->set_radius(FOOD_RADIUS);
  ++entity_count_;
  ++food_count_;
  food->set_id(food_count_);
  return food;
}

Pose EntityFactory::SetPoseRandomly() {
  // Dividing arena into 19x14 grid. Each grid square is 50x50
  return {static_cast<double>((30 + (random() % 19) * 50)),
        static_cast<double>((30 + (random() % 14) * 50))};
}

double EntityFactory::SetRadiusRandomlyLight() {
  // Returning the radius randomly.
  return static_cast<double>  (random() %
  (LIGHT_MAX_RADIUS + 1 - LIGHT_MIN_RADIUS) + LIGHT_MIN_RADIUS);
}

double EntityFactory::SetRadiusRandomlyRobot() {
  // returning a number between 8 and 14
  return static_cast<int> (8+(random()%(14-8+1)));
}

NAMESPACE_END(csci3081);
