#ifndef INC_PINCHDETECT_H_
#define INC_PINCHDETECT_H_

#include "can.h"
#include "main.h"
#include <stdint.h>

/* STM32 HAL I2C 함수에는 7비트 주소를 왼쪽으로 1비트 shift해서 전달한다. */
#define PINCHDETECT_ADDR_DRIVER     (0x40U << 1)
#define PINCHDETECT_ADDR_PASSENGER  (0x41U << 1)

typedef struct
{
    float shunt_mV;
    float bus_V;
    float current_A;
    uint8_t valid;
} PinchDetect_Data_t;

/* CubeIDE Watch 창 확인용 */
extern PinchDetect_Data_t driver_pinch_sensor;
extern PinchDetect_Data_t passenger_pinch_sensor;

void PinchDetect_Init(I2C_HandleTypeDef *hi2c);
void PinchDetect_Process(void);

uint8_t PinchDetect_IsDetected(SeatId_t seat);
void PinchDetect_Clear(SeatId_t seat);

/* 복구 동작 중에는 반대 방향으로 일부러 움직이므로,
 * 일정 시간 끼임 판단을 잠시 중지할 때 사용한다.
 */
void PinchDetect_Suspend(SeatId_t seat, uint32_t suspend_ms);

#endif /* INC_PINCHDETECT_H_ */
