#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>


#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


static uint Block = 0;
static uint16_t width = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;
static float distance_cm = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg)
{																		//Taken from the tp, and modified by us

    chRegSetThreadName(__FUNCTION__);									// We changed the height and the position of the line
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 380 + 381 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 380, IMAGE_BUFFER_SIZE, 3, SUBSAMPLING_X1, SUBSAMPLING_X1);
	po8030_set_brightness(20);											//change the brightness
	po8030_set_contrast(120);											//add contrast to help us reduce the noise
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1)
    {
//    	systime_t time = chVTGetSystemTime();
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
//		chprintf((BaseSequentialStream *)&SDU1, "Capture_time = %d\n ", chVTGetSystemTime()-time);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg)
{																//Taken form the tp, but modified by us
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};



    while(1)
    {
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		bool send_to_computer = true;

		for(uint16_t i=0; i< (2*IMAGE_BUFFER_SIZE); i +=2)
		{
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}
		if (send_to_computer)
		{
//			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		send_to_computer = !send_to_computer;

		Block = block_detection(image);
		palSetPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED7);
		distance_cm = PXTOCM/width;

		switch(Block)										//Indicate which post the robot sees, was used a lot during testing
		{													//We keep it because we find it decorative
			case RIGHT:
			{
				palClearPad(GPIOD, GPIOD_LED3);
				break;
			}
			case LEFT:
			{
				palClearPad(GPIOD, GPIOD_LED7);
				break;
			}
			case 0:
			{
				palClearPad(GPIOD, GPIOD_LED5);
				break;
			}
		}


    }
}

uint get_block(void)
{
	return Block;
}
uint16_t get_line_position(void)						//Taken form the tp
{
	return line_position;
}

uint16_t get_distance_cm(void)							//Taken form the tp
{
	return distance_cm;
}

void process_image_start(void)
{
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO+1, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO+1, CaptureImage, NULL);
}

uint block_detection(uint8_t *buffer)									//Function inspired by the tp, but totally remodeled
{																		//and remade by us
	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;
	uint block = 0;

	for(uint32_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++)
	{
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do
	{
		wrong_line = 0;
		while(stop == 0 && i< (IMAGE_BUFFER_SIZE))
		{
			if(buffer[i] > WHITE_VALUE && buffer[i-WIDTH_SLOPE] < WHITE_VALUE)			//Search for an positive slope, so a
			{																			//white stripe
				begin = i;
				stop = 1;
			}
			i++;
		}
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)								//Look for a negative slope,
		{																				//if there's a beginning
			stop = 0;

			while(stop == 0 && i < IMAGE_BUFFER_SIZE)
			{
				if(buffer[i] > WHITE_VALUE && buffer[i+WIDTH_SLOPE] < WHITE_VALUE)
				{
					end = i;
					line_position = (begin + end)/2;
					block = RIGHT;
				}
				i++;
				if(i<(IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && ((begin-end)<MIN_LINE_WIDTH))	//Start of the detection of which block is facing the robot
				{																		//by looking if there's another positive slope big enough
					if(buffer[i] > WHITE_VALUE && buffer[i-WIDTH_SLOPE] < WHITE_VALUE && buffer[i+MIN_LINE_WIDTH]> WHITE_VALUE)
					{
						begin = end;
						end = i;
						stop = 1;
						line_position = (begin + end)/2;
						block = LEFT;
					}
				}
			}
			if (i > IMAGE_BUFFER_SIZE || !end)
			{
				line_not_found = 1;
			}
		} else
		{
			line_not_found = 1;
		}
		if(!line_not_found &&  (((end-begin) < MIN_LINE_WIDTH) || ((end-begin) > MAX_LINE_WIDTH)))		//Check if the line found
		{																								//correspond to what we
			i = end;																					//are looking for
			begin = 0;
			end = 0;
			stop = 0;
			line_position = IMAGE_BUFFER_SIZE/2;
			width = 10;
			wrong_line = 1;
		}
	} while(wrong_line);
	if(line_not_found)
	{
		begin = 0;
		end = 0;
		block = 0;
		width = 10;
	} else
	{
		width =(end - begin);
	}
	return block;
}
