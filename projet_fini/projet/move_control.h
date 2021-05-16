
#ifndef MOVE_CONTROL_H_
#define MOVE_CONTROL_H_


#define DISTANCE_BETWEEN_BLOCKS 		4.8f		//This is the distance between the block in front of the robot and the center of the blocks
#define DISTANCE_TO_BLOCK				7			//This is the distance the robot have to travel after turning to be between the to blocks
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
* @param distance	The distance we want to move the robot. Here, the distance between the center of the blocks
* 					and the block either left or right
*
* @param block [LEFT RIGHT] The block the robot is facing.
*
*/
void move_between_blocks(uint block, float distance);
/**
* @brief   This a proportional regulator used in the control of the distance to the block using the camera.
*
* @param distance	The distance between the robot and the nearest block in front of it.
*
* @param block		This is the distance we want to reach before an action.
*
* @return			return the speed we have to input to the robot.
*
*/
int16_t pi_regulator_blocks(float distance, float goal);
/**
* @brief	This function is used to guide the robot between two blocks, using proximity sensors.
*
* @param capteur_right	this is the captor on the right of the robot we want to use.
*
* @param capteur_left	this is the captor on the leftt of the robot we want to use.
*
* @return			return the speed correction we have to input for the robot to adjust itself.
*
*/
int16_t regulator_capteurs(int capteur_right, int capteur_left);

#endif /* MOVE_CONTROL_H_ */
