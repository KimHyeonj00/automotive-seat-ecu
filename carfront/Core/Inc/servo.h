#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdint.h>

/* TIM2 CH1 = PA0, TIM2 CH2 = PA1 */
void Servo_Init(void);
void Servo_SetAngle(uint8_t channel, uint8_t angle);
void Servo_Process(void);
void Servo_EmergencyStop(uint8_t channel);
uint8_t Servo_IsMoving(uint8_t channel);
uint8_t Servo_GetCurrentAngle(uint8_t channel);
void Servo_Stop(void);

#endif /* INC_SERVO_H_ */
