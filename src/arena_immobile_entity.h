/**
 * @file arena_immobile_entity.h
 *
 * @copyright 2017 3081 Staff, All rights reserved.
 */

#ifndef SRC_ARENA_IMMOBILE_ENTITY_H_
#define SRC_ARENA_IMMOBILE_ENTITY_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "src/arena_entity.h"
#include "src/common.h"
#include "src/pose.h"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NAMESPACE_BEGIN(csci3081);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief An immobile entity in the Arena, incapable of moving.
 *
 * Immobile entities cannot move, and therefore do not need to override the
 * TimestepUpdate() function.
 */
class ArenaImmobileEntity : public ArenaEntity {
 public:
  /**
  * @brief ArenaImmobileEntity's constructor.
  *
  */
  ArenaImmobileEntity() : ArenaEntity() {}
};

NAMESPACE_END(csci3081);

#endif /* SRC_ARENA_IMMOBILE_ENTITY_H_ */
