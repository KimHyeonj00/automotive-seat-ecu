#ifndef INC_STEP_H_
#define INC_STEP_H_

#include <stdint.h>

/* 모터 1 = PA3~PA6, 모터 2 = PB12~PB15 */
void Step_Init(void);
void Step_SetTarget(uint8_t motor, int32_t target);
void Step_Move(uint8_t motor, int16_t steps);
void Step_Process(void);
void Step_Stop(uint8_t motor);
void Step_StopAll(void);
uint8_t Step_IsMoving(uint8_t motor);
int32_t Step_GetPosition(uint8_t motor);

#endif /* INC_STEP_H_ */
