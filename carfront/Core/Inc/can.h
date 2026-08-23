#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdint.h>

/* Front Zone ECU CAN IDs */
#define CAN_ID_SAFE_ABORT             0x010U
#define CAN_ID_GEAR_STATUS            0x070U
#define CAN_ID_DRIVER_SEAT_CMD        0x110U
#define CAN_ID_PASSENGER_SEAT_CMD     0x111U
#define CAN_ID_DRIVER_SEAT_STATUS     0x210U
#define CAN_ID_PASSENGER_SEAT_STATUS  0x211U

typedef enum
{
    SEAT_DRIVER = 0,
    SEAT_PASSENGER = 1,
    SEAT_COUNT = 2
} SeatId_t;

typedef struct
{
    SeatId_t seat;
    uint8_t recline_angle;
    uint8_t rotation_angle;
    uint8_t rolling_counter;
    uint8_t checksum_ok;
} SeatCommand_t;

typedef struct
{
    uint8_t gear;     /* 0=P, 1=D, 2=R. STM32에서는 저장만 함 */
} GearStatus_t;

typedef struct
{
    uint8_t stop_flag;
    uint8_t source_id;
    uint8_t reason_code;
} SafeAbort_t;

void CAN_AppStart(void);

uint8_t CAN_AppPopSeatCommand(SeatCommand_t *cmd);
uint8_t CAN_AppPopGearStatus(GearStatus_t *status);
uint8_t CAN_AppPopSafeAbort(SafeAbort_t *abort_msg);

void CAN_AppSendSeatStatus(SeatId_t seat,
                           uint8_t curr_recline,
                           uint8_t curr_rotation,
                           uint8_t pinch_detected);

#endif /* INC_CAN_H_ */
