
#ifndef MOVE_CONTROL_H_
#define MOVE_CONTROL_H_

void move_control_start(void);
void test_capteur(void);
/**
* @brief   Move the robot close to the block it is facing. Regulate it's speed and position with a Proportional Regulator.
*
*/
void move_to_block(void);
/**
* @brief   Turn the robot a quarter of a circle on himself. The direction depends on the block in front of him.
*
* @param block [LEFT RIGHT] The parameter to give is the block the robot is facing.
*
*/
void turn(uint block);
/**
* @brief   Will make the robot move in a straight line on the distance set on the paramater of the function.
*
* @param distance The distance we want to move the robot.
*
*/
void move_cm(float distance);
/**
* @brief   This function will move the robot form it's position facing the block it detected, to the center between the two blocks.
* 			It uses the functions turn and move_cm to accomplish it's goal.
*
* @param distance The distance we want to move the robot. Here, the distance between the center of the blocks
* 					and the block either left or right
*
* @param block [LEFT RIGHT] The block the robot is facing
*
*/
void move_between_blocks(uint block, float distance);

#endif /* MOVE_CONTROL_H_ */
