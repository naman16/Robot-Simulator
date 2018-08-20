/**
 * @file light.h
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

#ifndef SRC_LIGHT_H_
#define SRC_LIGHT_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "src/arena_mobile_entity.h"
#include "src/common.h"
#include "src/entity_type.h"
#include "src/pose.h"
#include "src/motion_handler.h"
#include "src/motion_behavior_differential.h"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief Class representing a mobile light within the Arena.
 *
 * Since lights are immobile, the Light class is very simple.
 */
class Light : public ArenaMobileEntity {
 public:
  /**
   * @brief Constructor.
   */
  Light();

  /**
   * @brief Get the name of the Light for visualization purposes, and to
   * aid in debugging.
   */

  /**
   * @brief Reset the Food using the initialization parameters received
   * by the constructor.
   */
  void Reset() override;


  std::string get_name() const override {
    return "Light" + std::to_string(get_id());
  }

  /**
   * @brief Update the Light's position and velocity after the specified
   * duration has passed.
   *
   * @param dt The # of timesteps that have elapsed since the last update.
   */
  void TimestepUpdate(unsigned int dt) override;

  /**
  * @brief Update the heading angle according to the touch sensor reading.
  */
  void UpdateVelocity();

   /**
   * @brief Update the position and orientation of Light
   * foodd on its current position and velocity.
   *
   * @param[in] dt # of timesteps elapsed since the last update.r
   */
  void UpdatePose(double dt);

  /**
   * @brief Handles the collision by setting the sensor to activated.
   *
   * @param[in] type of the object that the light collided with
   * @param[in] pointer to the object that the lights collided with
   */
  void HandleCollision(EntityType object_type, ArenaEntity * object = NULL);

  /**
  * @brief Generates radius for lights randomly and ensures that the radius
  * of light is random.
  */
  double SetRadiusRandomly() {
  // Returning the radius randomly.
  return static_cast<double>  (random() %
  (LIGHT_MAX_RADIUS + 1 - LIGHT_MIN_RADIUS) + LIGHT_MIN_RADIUS);
  }

   /**
   * @brief Changing the position, velocity and orientation of Light
   * when it collides with other entities or the wall and so that it can move
   * in an arc movement after collision for a fixed amount of time.
   *
   */
  void ArcMovement();

 private:
  // Manages pose and wheel velocities that change with time and collisions.
  MotionHandler motion_handler_;
  // Calculates changes in pose foodd on elapsed time and wheel velocities.
  MotionBehaviorDifferential motion_behavior_;
};

NAMESPACE_END(csci3081);

#endif  // SRC_LIGHT_H_
