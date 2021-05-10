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
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 200, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	po8030_set_awb(0);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
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
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};



    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		bool send_to_computer = true;

		/*
			To complete
		*/
		for(uint16_t i=0; i< (2*IMAGE_BUFFER_SIZE); i +=2)
		{
			image[i/2] = (uint8_t)img_buff_ptr[i]&0x1F8;
		}
		if (send_to_computer) {
//			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		send_to_computer = !send_to_computer;

		Block = block_detection(image);
		palSetPad(GPIOD, GPIOD_LED5);
		palSetPad(GPIOD, GPIOD_LED3);
		palSetPad(GPIOD, GPIOD_LED7);
		distance_cm = PXTOCM/width;

		switch(Block)
		{
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

uint get_block(void){
	return Block;
}
uint16_t get_line_position(void)
{
	return line_position;
}

uint16_t get_distance_cm(void)
{
	return distance_cm;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

uint block_detection(uint8_t *buffer)
{
	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;
//	bool left = 0;
	uint block = 0;
	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;
//	palClearPad(GPIOD, GPIOD_LED1);
//	palClearPad(GPIOD, GPIOD_LED3);
//	palClearPad(GPIOD, GPIOD_LED5);

	for(uint32_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++)
	{
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do
	{
		wrong_line = 0;
//		palSetPad(GPIOD, GPIOD_LED5); //test boucle dowhile
		while(stop == 0 && i< (IMAGE_BUFFER_SIZE))
		{
//			palSetPad(GPIOD, GPIOD_LED1); //test boucle1
			if(buffer[i] > 200 && buffer[i-WIDTH_SLOPE] < 200)
			{
				begin = i;
				stop = 1;
//				left = 0;
			}
			i++;
		}
//		palClearPad(GPIOD, GPIOD_LED1); //fin test 1
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
			stop = 0;

			while(stop == 0 && i < IMAGE_BUFFER_SIZE)
			{
//				palSetPad(GPIOD, GPIOD_LED3); //test boucle 2
				if(buffer[i] > 200 && buffer[i+WIDTH_SLOPE] < 200)
				{
					end = i;
//					stop = 1;
					line_position = (begin + end)/2;
					block = RIGHT;
				}
				i++;
				if(i<(IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && end)
				{
					if(buffer[i] > 200 && buffer[i-WIDTH_SLOPE] < 200)
					{
						begin = end;
						end = i;
						stop = 1;
						line_position = (begin + end)/2;
						block = LEFT;
					}
				}
			}
//			palClearPad(GPIOD, GPIOD_LED3); //fin test 2
			if (i > IMAGE_BUFFER_SIZE || !end)
			{
				line_not_found = 1;
			}
		} else
		{
			line_not_found = 1;
		}
		if(!line_not_found &&  (((end-begin) < MIN_LINE_WIDTH) || ((end-begin) > MAX_LINE_WIDTH) ||
				(line_position < IMAGE_BUFFER_SIZE/4) || (line_position > (3*IMAGE_BUFFER_SIZE/4))))
		{
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			line_position = IMAGE_BUFFER_SIZE/2;
			width = 0;
			wrong_line = 1;
		}
	} while(wrong_line);
//	palClearPad(GPIOD, GPIOD_LED5); //fin test dowhile
	if(line_not_found)
	{
		begin = 0;
		end = 0;
		block = 0;
		width = 0;
	} else
	{
		last_width = width =(end - begin);
//		line_position = (begin + end)/2;
	}
	return block;
}
