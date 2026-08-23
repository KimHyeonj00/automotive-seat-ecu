#include "servo.h"
#include "main.h"

#define SERVO_COUNT              2U

/* 0~180도 위치제어 서보 PWM 범위
 *
 * TIM2 설정:
 * Prescaler = 71
 * Period    = 19999
 *
 * 따라서 Compare 값 500  = 0.5ms
 * Compare 값 1500 = 1.5ms
 * Compare 값 2500 = 2.5ms
 *
 * 0~180도를 크게 보고 싶어서 500~2500us로 설정.
 * 끝단에서 서보가 윙- 하고 버티거나 떨리면 700~2300으로 줄여야 함.
 */
#define SERVO_MIN_US             500U
#define SERVO_MAX_US             2500U
#define SERVO_MAX_ANGLE          180U

/* 부드러운 이동 설정
 *
 * 10ms마다 PWM을 5us씩 변경.
 * 500~2500us 전체 이동은 약 4초 정도 걸림.
 *
 * 더 빠르게: SERVO_PULSE_STEP_US를 8~10으로 증가
 * 더 느리게: SERVO_PULSE_STEP_US를 2~3으로 감소
 */
#define SERVO_UPDATE_MS          10U
#define SERVO_PULSE_STEP_US      5U

extern TIM_HandleTypeDef htim2;

static uint16_t current_pulse[SERVO_COUNT];
static uint16_t target_pulse[SERVO_COUNT];
static uint8_t enabled[SERVO_COUNT];
static uint32_t last_update_tick[SERVO_COUNT];

static uint32_t channel_to_tim(uint8_t channel)
{
    return (channel == 1U) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
}

static uint16_t angle_to_pulse(uint8_t angle)
{
    if (angle > SERVO_MAX_ANGLE)
    {
        angle = SERVO_MAX_ANGLE;
    }

    return (uint16_t)(SERVO_MIN_US +
           (((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US)) / SERVO_MAX_ANGLE));
}

static uint8_t pulse_to_angle(uint16_t pulse)
{
    if (pulse <= SERVO_MIN_US)
    {
        return 0U;
    }

    if (pulse >= SERVO_MAX_US)
    {
        return SERVO_MAX_ANGLE;
    }

    return (uint8_t)((((uint32_t)(pulse - SERVO_MIN_US)) * SERVO_MAX_ANGLE) /
                     (SERVO_MAX_US - SERVO_MIN_US));
}

static void servo_start(uint8_t channel)
{
    uint8_t index;

    if (channel < 1U || channel > SERVO_COUNT)
    {
        return;
    }

    index = channel - 1U;

    if (enabled[index] == 0U)
    {
        HAL_TIM_PWM_Start(&htim2, channel_to_tim(channel));
        enabled[index] = 1U;
    }
}

static void servo_write_pulse(uint8_t channel, uint16_t pulse)
{
    if (channel < 1U || channel > SERVO_COUNT)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(&htim2, channel_to_tim(channel), pulse);
}

void Servo_Init(void)
{
    uint8_t i;
    uint16_t center_pulse = angle_to_pulse(90U);

    for (i = 0U; i < SERVO_COUNT; i++)
    {
        current_pulse[i] = center_pulse;
        target_pulse[i] = center_pulse;
        enabled[i] = 0U;
        last_update_tick[i] = 0U;
    }
}

void Servo_SetAngle(uint8_t channel, uint8_t angle)
{
    uint8_t index;

    if (channel < 1U || channel > SERVO_COUNT)
    {
        return;
    }

    if (angle > SERVO_MAX_ANGLE)
    {
        angle = SERVO_MAX_ANGLE;
    }

    index = channel - 1U;

    target_pulse[index] = angle_to_pulse(angle);

    servo_start(channel);
    servo_write_pulse(channel, current_pulse[index]);

    last_update_tick[index] = HAL_GetTick();
}

void Servo_Process(void)
{
    uint8_t i;
    uint8_t channel;
    uint32_t now = HAL_GetTick();

    for (i = 0U; i < SERVO_COUNT; i++)
    {
        if (enabled[i] == 0U)
        {
            continue;
        }

        if ((now - last_update_tick[i]) < SERVO_UPDATE_MS)
        {
            continue;
        }

        last_update_tick[i] = now;
        channel = (uint8_t)(i + 1U);

        if (current_pulse[i] < target_pulse[i])
        {
            if ((uint16_t)(target_pulse[i] - current_pulse[i]) <= SERVO_PULSE_STEP_US)
            {
                current_pulse[i] = target_pulse[i];
            }
            else
            {
                current_pulse[i] += SERVO_PULSE_STEP_US;
            }

            servo_write_pulse(channel, current_pulse[i]);
        }
        else if (current_pulse[i] > target_pulse[i])
        {
            if ((uint16_t)(current_pulse[i] - target_pulse[i]) <= SERVO_PULSE_STEP_US)
            {
                current_pulse[i] = target_pulse[i];
            }
            else
            {
                current_pulse[i] -= SERVO_PULSE_STEP_US;
            }

            servo_write_pulse(channel, current_pulse[i]);
        }
        else
        {
            /* 목표 위치 도착 후에도 PWM 유지.
             * 위치제어 서보는 PWM을 유지해야 해당 각도를 잡고 있음.
             */
            servo_write_pulse(channel, current_pulse[i]);
        }
    }
}

void Servo_EmergencyStop(uint8_t channel)
{
    uint8_t index;

    if (channel < 1U || channel > SERVO_COUNT)
    {
        return;
    }

    index = channel - 1U;
    target_pulse[index] = current_pulse[index];
    servo_write_pulse(channel, current_pulse[index]);
}

uint8_t Servo_IsMoving(uint8_t channel)
{
    uint8_t index;

    if (channel < 1U || channel > SERVO_COUNT)
    {
        return 0U;
    }

    index = channel - 1U;

    return (current_pulse[index] != target_pulse[index]) ? 1U : 0U;
}

uint8_t Servo_GetCurrentAngle(uint8_t channel)
{
    if (channel < 1U || channel > SERVO_COUNT)
    {
        return 0U;
    }

    return pulse_to_angle(current_pulse[channel - 1U]);
}

void Servo_Stop(void)
{
    Servo_EmergencyStop(1U);
    Servo_EmergencyStop(2U);
}
