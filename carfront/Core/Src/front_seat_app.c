#include "front_seat_app.h"
#include "can.h"
#include "pinchdetect.h"
#include "servo.h"
#include "step.h"

#define SEAT_MAX_ANGLE_DEG            180U
#define STATUS_PERIOD_MS              100U

/* 회전 스텝모터 튜닝값 */
#define DRIVER_ROTATION_180_STEPS      (-2048L)
#define PASSENGER_ROTATION_180_STEPS   2048L

/* 끼임 발생 후 복구 동작 튜닝값
 * 1) 끼임 발생 즉시 정지
 * 2) 가던 방향의 반대 방향으로 PINCH_BACKOFF_DEG만큼 이동
 * 3) PINCH_RETRY_WAIT_MS만큼 대기
 * 4) 원래 명령 위치로 1회 재시도
 * 5) 재시도 중 다시 끼이면 LOCK 상태로 정지 유지
 */
#define PINCH_BACKOFF_DEG             30U
#define PINCH_RETRY_WAIT_MS           1000U
#define PINCH_SUSPEND_MS              500U
#define PINCH_MAX_RETRY               1U

typedef struct
{
    uint8_t servo_ch;
    uint8_t step_motor;
    int32_t steps_at_180deg;
} SeatHwMap_t;

typedef enum
{
    SEAT_CTRL_IDLE = 0,
    SEAT_CTRL_MOVING,
    SEAT_CTRL_BACKOFF,
    SEAT_CTRL_WAIT_RETRY,
    SEAT_CTRL_RETRY_TO_TARGET,
    SEAT_CTRL_LOCKED_BY_PINCH
} SeatCtrlState_t;

typedef struct
{
    uint8_t target_recline;
    uint8_t target_rotation;
    uint8_t backoff_recline;
    int8_t recline_dir;        /* +1: 각도 증가 방향, -1: 각도 감소 방향 */
    uint8_t retry_count;
    uint32_t wait_start_tick;
    SeatCtrlState_t state;
} SeatCtrl_t;

typedef struct
{
    uint8_t gear;              /* 수신 저장용. 제어 허용 판단은 Raspberry Pi에서 수행 */
    uint8_t safe_abort_active;
    uint32_t last_status_tick;
    SeatCtrl_t ctrl[SEAT_COUNT];
} FrontSeatContext_t;

static const SeatHwMap_t seat_hw[SEAT_COUNT] =
{
    [SEAT_DRIVER]    = { 1U, 1U, DRIVER_ROTATION_180_STEPS },
    [SEAT_PASSENGER] = { 2U, 2U, PASSENGER_ROTATION_180_STEPS }
};

static FrontSeatContext_t app;

static uint8_t seat_is_valid(SeatId_t seat)
{
    return ((uint8_t)seat < SEAT_COUNT) ? 1U : 0U;
}

static uint8_t cmd_is_valid(const SeatCommand_t *cmd)
{
    if (cmd == 0 || seat_is_valid(cmd->seat) == 0U)
    {
        return 0U;
    }

    if (cmd->checksum_ok == 0U)
    {
        return 0U;
    }

    /* DBC 범위 밖 값은 180도로 보정하지 않고 무시한다. */
    if (cmd->recline_angle > SEAT_MAX_ANGLE_DEG ||
        cmd->rotation_angle > SEAT_MAX_ANGLE_DEG)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t clamp_angle_i16(int16_t angle)
{
    if (angle < 0)
    {
        return 0U;
    }

    if (angle > 180)
    {
        return 180U;
    }

    return (uint8_t)angle;
}

static int32_t angle_to_steps(SeatId_t seat, uint8_t angle_deg)
{
    return (seat_hw[seat].steps_at_180deg * (int32_t)angle_deg) / 180L;
}

static uint8_t steps_to_angle(SeatId_t seat, int32_t steps)
{
    int32_t steps_180 = seat_hw[seat].steps_at_180deg;
    int32_t angle;

    if (steps_180 == 0L)
    {
        return 0U;
    }

    angle = (steps * 180L) / steps_180;

    if (angle < 0L)
    {
        angle = 0L;
    }
    else if (angle > 180L)
    {
        angle = 180L;
    }

    return (uint8_t)angle;
}

static uint8_t current_recline_angle(SeatId_t seat)
{
    return Servo_GetCurrentAngle(seat_hw[seat].servo_ch);
}

static uint8_t current_rotation_angle(SeatId_t seat)
{
    return steps_to_angle(seat, Step_GetPosition(seat_hw[seat].step_motor));
}

static uint8_t seat_is_busy(SeatId_t seat)
{
    const SeatHwMap_t *hw = &seat_hw[seat];

    if (Servo_IsMoving(hw->servo_ch) != 0U)
    {
        return 1U;
    }

    if (Step_IsMoving(hw->step_motor) != 0U)
    {
        return 1U;
    }

    return 0U;
}

static void send_status(SeatId_t seat)
{
    CAN_AppSendSeatStatus(seat,
                          current_recline_angle(seat),
                          current_rotation_angle(seat),
                          PinchDetect_IsDetected(seat));
}

static void send_all_status(void)
{
    send_status(SEAT_DRIVER);
    send_status(SEAT_PASSENGER);
}

static void send_periodic_status(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - app.last_status_tick) < STATUS_PERIOD_MS)
    {
        return;
    }

    app.last_status_tick = now;
    send_all_status();
}

static void stop_one_seat(SeatId_t seat)
{
    Servo_EmergencyStop(seat_hw[seat].servo_ch);
    Step_Stop(seat_hw[seat].step_motor);
}

static void stop_all_seats(void)
{
    Servo_Stop();
    Step_StopAll();
}

static void init_one_ctrl(SeatId_t seat)
{
    app.ctrl[seat].target_recline = current_recline_angle(seat);
    app.ctrl[seat].target_rotation = current_rotation_angle(seat);
    app.ctrl[seat].backoff_recline = app.ctrl[seat].target_recline;
    app.ctrl[seat].recline_dir = 0;
    app.ctrl[seat].retry_count = 0U;
    app.ctrl[seat].wait_start_tick = 0U;
    app.ctrl[seat].state = SEAT_CTRL_IDLE;
}

static void apply_seat_command(const SeatCommand_t *cmd)
{
    const SeatHwMap_t *hw = &seat_hw[cmd->seat];
    SeatCtrl_t *ctrl = &app.ctrl[cmd->seat];
    uint8_t curr_recline;

    curr_recline = current_recline_angle(cmd->seat);

    ctrl->target_recline = cmd->recline_angle;
    ctrl->target_rotation = cmd->rotation_angle;
    ctrl->backoff_recline = curr_recline;
    ctrl->retry_count = 0U;
    ctrl->wait_start_tick = 0U;
    ctrl->state = SEAT_CTRL_MOVING;

    if (cmd->recline_angle > curr_recline)
    {
        ctrl->recline_dir = 1;
    }
    else if (cmd->recline_angle < curr_recline)
    {
        ctrl->recline_dir = -1;
    }
    else
    {
        ctrl->recline_dir = 0;
    }

    PinchDetect_Clear(cmd->seat);

    Servo_SetAngle(hw->servo_ch, cmd->recline_angle);
    Step_SetTarget(hw->step_motor, angle_to_steps(cmd->seat, cmd->rotation_angle));
}

static void handle_safe_abort(const SafeAbort_t *abort_msg)
{
    uint8_t i;

    app.safe_abort_active = (abort_msg->stop_flag != 0U) ? 1U : 0U;
    stop_all_seats();

    for (i = 0U; i < SEAT_COUNT; i++)
    {
        init_one_ctrl((SeatId_t)i);
    }

    send_all_status();
}

static void process_can_events(void)
{
    SeatCommand_t cmd;
    GearStatus_t gear;
    SafeAbort_t abort_msg;

    if (CAN_AppPopSafeAbort(&abort_msg) != 0U)
    {
        handle_safe_abort(&abort_msg);
    }

    if (CAN_AppPopGearStatus(&gear) != 0U)
    {
        app.gear = gear.gear;
    }

    while (CAN_AppPopSeatCommand(&cmd) != 0U)
    {
        if (app.safe_abort_active != 0U)
        {
            continue;
        }

        if (cmd_is_valid(&cmd) == 0U)
        {
            continue;
        }

        apply_seat_command(&cmd);
        send_status(cmd.seat);
    }
}

static void process_motion(void)
{
    if (app.safe_abort_active != 0U)
    {
        return;
    }

    Servo_Process();
    Step_Process();
}

static int8_t decide_backoff_dir(SeatId_t seat, const SeatCtrl_t *ctrl)
{
    uint8_t curr_recline;

    if (ctrl->recline_dir != 0)
    {
        return ctrl->recline_dir;
    }

    curr_recline = current_recline_angle(seat);

    if (ctrl->target_recline > curr_recline)
    {
        return 1;
    }

    if (ctrl->target_recline < curr_recline)
    {
        return -1;
    }

    /* 목표각과 현재각이 같으면 기본적으로 감소 방향으로 가던 것으로 가정 */
    return -1;
}

static void start_pinch_backoff(SeatId_t seat)
{
    SeatCtrl_t *ctrl = &app.ctrl[seat];
    const SeatHwMap_t *hw = &seat_hw[seat];
    uint8_t curr_recline;
    int8_t dir;
    int16_t backoff_angle;

    /* 지금 상태를 먼저 1회 보고한다. 이때 pinch bit = 1 */
    send_status(seat);

    if (ctrl->retry_count >= PINCH_MAX_RETRY)
    {
        ctrl->state = SEAT_CTRL_LOCKED_BY_PINCH;
        stop_one_seat(seat);
        return;
    }

    curr_recline = current_recline_angle(seat);
    dir = decide_backoff_dir(seat, ctrl);

    /* 가던 방향의 반대 방향으로 PINCH_BACKOFF_DEG만큼 빼기 */
    backoff_angle = (int16_t)curr_recline - ((int16_t)dir * (int16_t)PINCH_BACKOFF_DEG);
    ctrl->backoff_recline = clamp_angle_i16(backoff_angle);
    ctrl->retry_count++;

    PinchDetect_Clear(seat);
    PinchDetect_Suspend(seat, PINCH_SUSPEND_MS);

    Servo_SetAngle(hw->servo_ch, ctrl->backoff_recline);
    Step_Stop(hw->step_motor);

    ctrl->state = SEAT_CTRL_BACKOFF;
    send_status(seat);
}

static void process_pinch_recovery(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t i;

    if (app.safe_abort_active != 0U)
    {
        return;
    }

    for (i = 0U; i < SEAT_COUNT; i++)
    {
        SeatId_t seat = (SeatId_t)i;
        SeatCtrl_t *ctrl = &app.ctrl[i];
        const SeatHwMap_t *hw = &seat_hw[i];

        if (PinchDetect_IsDetected(seat) != 0U)
        {
            start_pinch_backoff(seat);
            continue;
        }

        switch (ctrl->state)
        {
            case SEAT_CTRL_MOVING:
                if (seat_is_busy(seat) == 0U)
                {
                    ctrl->state = SEAT_CTRL_IDLE;
                }
                break;

            case SEAT_CTRL_BACKOFF:
                if (Servo_IsMoving(hw->servo_ch) == 0U)
                {
                    ctrl->wait_start_tick = now;
                    ctrl->state = SEAT_CTRL_WAIT_RETRY;
                }
                break;

            case SEAT_CTRL_WAIT_RETRY:
                if ((now - ctrl->wait_start_tick) >= PINCH_RETRY_WAIT_MS)
                {
                    PinchDetect_Suspend(seat, PINCH_SUSPEND_MS);

                    Servo_SetAngle(hw->servo_ch, ctrl->target_recline);
                    Step_SetTarget(hw->step_motor, angle_to_steps(seat, ctrl->target_rotation));

                    ctrl->state = SEAT_CTRL_RETRY_TO_TARGET;
                    send_status(seat);
                }
                break;

            case SEAT_CTRL_RETRY_TO_TARGET:
                if (seat_is_busy(seat) == 0U)
                {
                    ctrl->state = SEAT_CTRL_IDLE;
                }
                break;

            case SEAT_CTRL_LOCKED_BY_PINCH:
            case SEAT_CTRL_IDLE:
            default:
                break;
        }
    }
}

void FrontSeatApp_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t i;

    app.gear = 0U;
    app.safe_abort_active = 0U;
    app.last_status_tick = HAL_GetTick();

    Servo_Init();
    Step_Init();
    PinchDetect_Init(hi2c);
    CAN_AppStart();

    for (i = 0U; i < SEAT_COUNT; i++)
    {
        init_one_ctrl((SeatId_t)i);
    }

    send_all_status();
}

void FrontSeatApp_Process(void)
{
    process_can_events();
    process_motion();
    PinchDetect_Process();
    process_pinch_recovery();
    send_periodic_status();
}
