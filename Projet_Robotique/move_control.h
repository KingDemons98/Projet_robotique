
#ifndef MOVE_CONTROL_H_
#define MOVE_CONTROL_H_

void move_control_start(void);
void test_capteur(void);
void move_to_block(int16_t speed, int16_t speed_correction);
void turn(uint block, int32_t count_right);

#endif /* MOVE_CONTROL_H_ */
