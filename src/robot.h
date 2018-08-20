/**
 * @file robot.h
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "src/arena_mobile_entity.h"
#include "src/common.h"
#include "src/motion_handler_robot.h"
#include "src/motion_behavior_differential.h"
#include "src/entity_type.h"
#include "src/light_sensor.h"
#include "src/food_sensor.h"
#include "src/motion_handler_aggression.h"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

class MotionBehaviorDifferential;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief Class representing a robot within the arena.
 *
 * Robots are composed of a motion handler, motion behavior, and touch sensor.
 * These classes interact to maintain the pose (position and heading) of the
 * robot. At each time step, the wheel velocities are used to calculate the
 * next pose of the robot. The handler manages the pose and user requests. The
 * behavior calculates the new pose foodd on wheel velocities.
 *
 * Robots can be controlled through keypress, which modify wheel velocities.
 *
 * The touch sensor is activated when the robot collides with an object.
 * The heading is modified after a collision to move the robot away from the
 * other object.
 */
class Robot : public ArenaMobileEntity {
 public:
  /**
   * @brief Constructor using initialization values from params.h.
   */

  explicit Robot(int behavior);
  Robot(const Robot& other) = default;
  Robot& operator=(const Robot& other) = default;

  /**
   * @brief Default destructor.
   */
  virtual ~Robot() = default;

  /**
   * @brief Reset the Robot to a newly constructed state (needed for reset
   * button to work in GUI).
   */
  void Reset() override;

  /**
   * @brief Update the Robot's position and velocity after the specified
   * duration has passed.
   *
   * @param dt The # of timesteps that have elapsed since the last update.
   */
  void TimestepUpdate(unsigned int dt) override;

  /**
   * @brief This method checks for what behavior state the robot is in
   * i.e. if it is arcing (since it has undergone collision) or hungry and
   * then accordingly takes the requiired action by updating the time counter.
   *
   * If the robot is very hungry, then this method will make the robot blink
   * to show that state.
   * If the robot has just undergone collision, this method will then call the
   * ArcMovement method while the robot continues to be in the arcing state.
   * Finally,if the food has been turned off, then this method makes sure that
   * the robot will never be in a hungry state.
   */
  void RobotStateUpdate();
  /**
   * @brief Handles the collision by setting the sensor to activated.
   *
   * @param[in] object_type Type of the entity that the Robot has collided with.
   * @param[in] object Instance of the entity that the robot has collided with.
   */
  void HandleCollision(EntityType object_type, ArenaEntity * object = NULL);

  /**
   * @brief Get the name of the Robot for visualization and for debugging.
   */
  std::string get_name() const override { return "Robot"; }

  /**
   * @brief Command that comes from the controller, then is passed to handler.
   */
  void IncreaseSpeed();

  /**
  * @brief Command that comes from the controller, then is passed to handler.
  */
  void DecreaseSpeed();

  /**
  * @brief Command that comes from the controller, then is passed to handler.
  */
  void TurnRight();

  /**
  * @brief Command that comes from the controller, then is passed to handler.
  */
  void TurnLeft();

  int get_lives() const { return lives_; }

  void set_lives(int l) { lives_ = l; }

  MotionHandlerRobot *get_motion_handler() {return motion_handler_;}
  MotionBehaviorDifferential *get_motion_behavior() {return &motion_behavior_;}
  /**
  * @brief Generates radius for robots randomly in the range 8-14
  * and ensures that the radius of robot is random.
  */
  double SetRadiusRandomlyRobot() {
    // returning a number between 8 and 14
    return static_cast<int> (8+(random()%(14-8+1)));
  }

  /**
  * @brief Updating the left sensor position at each timestep.
  */
  Pose PoseLeftSensor() const {
    double angle_ = (this)->get_heading() + (-40.0*M_PI/180.0);
    double x_ = (this)->get_radius()*cos(angle_) + (this)->get_pose().x;
    double y_ = (this)->get_radius()*sin(angle_) + (this)->get_pose().y;
    return {x_, y_};
  }
  /**
  * @brief Updating the right sensor position at each timestep.
  */
  Pose PoseRightSensor() const {
    double angle_ = (this)->get_heading() + (40.0*M_PI/180.0);
    double x_ = (this)->get_radius()*cos(angle_) + (this)->get_pose().x;
    double y_ = (this)->get_radius()*sin(angle_) + (this)->get_pose().y;
    return {x_, y_};
  }

  /**
  * @brief Giving Robot its arc movement.
  */
  void ArcMovement();

  /**
  * @brief Getter for the left light sensor.
  */
  LightSensor *get_left_light_sensor() { return &left_light_sensor_;}
  /**
  * @brief Getter for the right light sensor.
  */
  LightSensor *get_right_light_sensor() { return &right_light_sensor_;}
  /**
  * @brief Getter for the left food sensor.
  */
  FoodSensor *get_left_food_sensor() { return &left_food_sensor_; }
  /**
  * @brief Getter for the right food sensor.
  */
  FoodSensor *get_right_food_sensor() { return &right_food_sensor_; }
  /**
  * @brief Returns true if the robot is very hungry i.e. robot hasn't
  * consumed food for 2 mins.
  */
  bool RobotReallyHungry();
  /**
  * @brief Returns true if the robot is starving i.e. robot hasn't consumed food
  * for 2.5 mins.
  */
  bool RobotStarving();

  /**
  * @brief Robot gets the entity from the arena and then based on its behavior
  * and sensor reading value for that entity, robot updates its velocity
  * accordingly.
  *
  * @param Type of the entity causing robot to update its velocity.
  * @param The instance of the entity itself causing the update.
  */
  void RobotDecideMotion(EntityType object_type, ArenaEntity * object = NULL);

  /**
   * @brief Determine if the robot has consumed that food entity. If the robot
   * is within 5 pixels of the food, then it means it has consumed the food.
   * @param other_e This entity is the food.
   * @param[out] True if food is consumed.
   *
  **/
    bool IsFoodConsumed(ArenaEntity * const other_e);

    /**
    *
    * @brief Setter for the behavior_light_flag_ of robot.
    */
    void set_behavior_flag(int lbehavior) { behavior_light_flag_ = lbehavior; }

    /**
    * @brief Function returns the behavior light flag that is if the robot
    * fears light or explores light. if the flag value is even, then robot
    * explores light and if flag is odd, robot fears light.
    */
    int get_behavior_flag() const { return behavior_light_flag_;}

    /**
    * @brief Setter for the sensitivity of the robot towards light.
    *
    * @param[in] lsensor_base The base value as provided by the user for
    * sensitivity towards light.
    */
    void set_sensitivity_to_light(float lsensor_base);
    /**
    * @brief Setter for letting the robot know if food is on or off.
    *
    * @param[in] fonoff The value passed in by the user for food on/off.
    * If fonoff is 1, then food is on else food is off if fonoff is 0.
    */
    void set_food_on_off(int fonoff) {
      food_on_off_ = fonoff;
    }
    /**
    * @brief Setter for ReallyHungry only needed for the unit tests.
    *
    * @param[in] reallyhungry Specifies very hungry or not
    */
    void set_really_hungry(bool reallyhungry) {
      really_hungry_ = reallyhungry;
  }

 private:
  // Manages pose and wheel velocities that change with time and collisions.
  MotionHandlerRobot* motion_handler_;
  // Calculates changes in pose foodd on elapsed time and wheel velocities.
  MotionBehaviorDifferential motion_behavior_;
  // Lives are decremented when the robot collides with anything.
  // When all the lives are gone, the game is lost.
  int lives_;
  // Keeps track of the time that has elapsed since last meal.
  int time_counter_;
  // Keeps track of if robot has collided with light or not and this is used
  // to implement mercy invincibility.
  bool collision_tracker_;
  // Manages the left and right light sensors.
  LightSensor right_light_sensor_;
  LightSensor left_light_sensor_;
  // Manages the left and right food senors.
  FoodSensor right_food_sensor_;
  FoodSensor left_food_sensor_;
  // Keeping track of behavior of robot towards light,
  // even mean exploratory behavior, odd means fearful behavior
  int behavior_light_flag_;
  // Keeps track of if robot is hungry or not.
  bool hunger_tracker_;
  // Keeps track of the time passed between collision.
  int collision_timer_;
  // Sensitivity to light
  float sensitivity_to_light_;
  // food on or off, 1 means on, 0 means off
  int food_on_off_;
  // ReallyHungry flag
  bool really_hungry_;
};

NAMESPACE_END(csci3081);

#endif  // SRC_ROBOT_H_
