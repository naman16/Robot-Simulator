/**
 * @file communication.h
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

#ifndef SRC_COMMUNICATION_H_
#define SRC_COMMUNICATION_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "src/common.h"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/**
* @brief Contains all the commands that are required to facilitate communication
* between GUI, Contoller and Arena.
*/

enum Communication {
  // communications from GUI to controller
  kKeyUp,
  kKeyDown,
  kKeyRight,
  kKeyLeft,
  kPlay,
  kPause,
  kNewGame,

  // communications from controller to Arena
  kIncreaseSpeed,
  kDecreaseSpeed,
  kTurnRight,
  kTurnLeft,
  kReset,

  // communications from Arena to Controller
  kWon,
  kLost,

  kNone   // in case it is needed
};

NAMESPACE_END(csci3081);

#endif  // SRC_COMMUNICATION_H_
