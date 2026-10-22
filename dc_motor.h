#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

#include <stdint.h>

#define DC_MOTOR_PWM_MAX               6000  /* 最终PWM输出限幅，满量程9999 */
#define DC_MOTOR_TEST_PWM              1500
#define DC_MOTOR_DIR_L                 -1
#define DC_MOTOR_DIR_R                 1

#if ((DC_MOTOR_DIR_L != 1) && (DC_MOTOR_DIR_L != -1))
#error "DC_MOTOR_DIR_L must be 1 or -1"
#endif

#if ((DC_MOTOR_DIR_R != 1) && (DC_MOTOR_DIR_R != -1))
#error "DC_MOTOR_DIR_R must be 1 or -1"
#endif

void dc_motor_init(void);
void dc_motor_set_speed_l(int16_t pwm);
void dc_motor_set_speed_r(int16_t pwm);
int16_t dc_motor_get_pwm_l(void);
int16_t dc_motor_get_pwm_r(void);
void dc_motor_stop(void);

#endif /* DC_MOTOR_H_ */
