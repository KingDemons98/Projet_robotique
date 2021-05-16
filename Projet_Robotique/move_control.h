
#ifndef MOVE_CONTROL_H_
#define MOVE_CONTROL_H_


#define DISTANCE_BETWEEN_BLOCKS 		4.8f
#define DISTANCE_TO_BLOCK				7
#define QUARTER_TURN					0.25f

/**
* @brief   Start the thread moving the robot.
*
*/
void move_control_start(void);
/**
* @brief   Start the thread responsible for stopping the robot once it's done with the slope.
*
*/
void imu_ending_start(void);
/**
* @brief   Move the robot close to the block it is facing. Regulate it's speed and position with a Proportional Regulator.
*
*/
void move_to_block(void);
/**
* @brief   Turn the robot a quarter of a circle on himself. The direction depends on the block in front of him. Can be used to
* 			turn the robot without block by giving him a direction. RIGHT will make it turn counter-clockwise and LEFT clockwise.
*
* @param block [LEFT RIGHT] The parameter to give is the block the robot is facing.
*
* @param number_turns 	the number of turns we want the robot to do. Can be a fraction of a turn
*
*/
void turn(uint block, float number_turns);
/**
* @brief   Will make the robot move in a straight line on the distance set on the parameter of the function.
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
* @param block [LEFT RIGHT] The block the robot is facing.
*
*/
void move_between_blocks(uint block, float distance);

#endif /* MOVE_CONTROL_H_ */
