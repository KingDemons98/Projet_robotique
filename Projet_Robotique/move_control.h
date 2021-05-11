
#ifndef MOVE_CONTROL_H_
#define MOVE_CONTROL_H_

void move_control_start(void);
void test_capteur(void);
void move_to_block(int16_t speed, int16_t speed_correction);
void turn(uint block);
void move_cm(uint distance);

#endif /* MOVE_CONTROL_H_ */
