#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

uint get_block(void);
void process_image_start(void);
uint16_t get_distance_cm(void);
uint block_detection(uint8_t *buffer);
uint16_t get_line_position(void);

#endif /* PROCESS_IMAGE_H */
