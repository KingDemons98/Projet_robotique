#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

/***************************INTERNAL FUNCTIONS************************************/
/**
* @brief	return which block is facing the robot.
*
* @param	buffer		pointer to a buffer containing the images taken by the camera
*
* @return	the block facing the robot, can be RIGHT, LEFT or 0.
*/
uint block_detection(uint8_t *buffer);
/*************************END INTERNAL FUNCTIONS**********************************/

/****************************PUBLIC FUNCTIONS*************************************/
 /**
 * @brief   Starts The captures of the images.
 */
void process_image_start(void);
/**
* @brief	return which block is facing the robot. Used for other modules
*
* @return	the block facing the robot, can be RIGHT, LEFT or 0.
*/
uint get_block(void);
/**
* @brief	return the distance form the robot to the block in front of it.
*
* @return	the distance in centimenter to the nearest block.
*/
uint16_t get_distance_cm(void);
/**
* @brief   return the position of the line on the block in front of the robot.
*
* @return	the position of the line measured and calculated.
*/
uint16_t get_line_position(void);


#endif /* PROCESS_IMAGE_H */
