#include "step.h"
#include "main.h"

#define STEP_MOTOR_COUNT      2U
#define STEP_INTERVAL_MS      5U

/* ULN2003 드라이버로 28BYJ-48을 돌리기 위한 half-step 시퀀스 */
static const uint8_t half_step[8][4] =
{
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

static int32_t position[STEP_MOTOR_COUNT];
static int32_t target[STEP_MOTOR_COUNT];
static uint8_t seq_index[STEP_MOTOR_COUNT];
static uint32_t last_step_tick;

/* 선택한 모터의 ULN2003 입력핀에 half-step 패턴 1개를 출력한다.
 * 모터 1: PA3, PA4, PA5, PA6
 * 모터 2: PB12, PB13, PB14, PB15
 */
static void step_write(uint8_t motor, uint8_t pattern)
{
    GPIO_TypeDef *port;
    uint16_t p0;
    uint16_t p1;
    uint16_t p2;
    uint16_t p3;

    if (motor == 1U)
    {
        port = GPIOA;
        p0 = GPIO_PIN_6;
        p1 = GPIO_PIN_5;
        p2 = GPIO_PIN_4;
        p3 = GPIO_PIN_3;
    }
    else if (motor == 2U)
    {
        port = GPIOB;
        p0 = GPIO_PIN_15;
        p1 = GPIO_PIN_14;
        p2 = GPIO_PIN_13;
        p3 = GPIO_PIN_12;
    }
    else
    {
        return;
    }

    HAL_GPIO_WritePin(port, p0, half_step[pattern][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port, p1, half_step[pattern][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port, p2, half_step[pattern][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port, p3, half_step[pattern][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void step_release(uint8_t motor)
{
    /* 목표 위치에 도달하거나 정지할 때 코일 전원을 끊어 발열을 줄인다. */
    if (motor == 1U)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
    }
    else if (motor == 2U)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
    }
}

void Step_Init(void)
{
    uint8_t i;

    /* 홈 센서가 없으므로 전원 켜질 때의 위치를 0으로 가정한다. */
    for (i = 0U; i < STEP_MOTOR_COUNT; i++)
    {
        position[i] = 0;
        target[i] = 0;
        seq_index[i] = 0;
    }

    step_release(1);
    step_release(2);
}

void Step_SetTarget(uint8_t motor, int32_t new_target)
{
    /* half-step 기준 절대 목표 위치를 설정한다. */
    if (motor < 1U || motor > STEP_MOTOR_COUNT)
    {
        return;
    }

    target[motor - 1U] = new_target;
}

void Step_Move(uint8_t motor, int16_t steps)
{
    /* 현재 목표 위치에서 상대 이동량을 더한다. 수동 조그 테스트에 사용하기 좋다. */
    if (motor < 1U || motor > STEP_MOTOR_COUNT)
    {
        return;
    }

    target[motor - 1U] += steps;
}

void Step_Process(void)
{
    uint8_t i;
    uint32_t now = HAL_GetTick();

    if ((now - last_step_tick) < STEP_INTERVAL_MS)
    {
        /* 논블로킹 제어라서 main loop가 계속 돌 수 있다. */
        return;
    }
    last_step_tick = now;

    for (i = 0U; i < STEP_MOTOR_COUNT; i++)
    {
        if (position[i] < target[i])
        {
            /* 정방향으로 half-step 1개 이동 */
            seq_index[i] = (uint8_t)((seq_index[i] + 1U) & 0x07U);
            position[i]++;
            step_write((uint8_t)(i + 1U), seq_index[i]);
        }
        else if (position[i] > target[i])
        {
            /* 역방향으로 half-step 1개 이동 */
            seq_index[i] = (uint8_t)((seq_index[i] + 7U) & 0x07U);
            position[i]--;
            step_write((uint8_t)(i + 1U), seq_index[i]);
        }
        else
        {
            /* 목표 위치에 도달하면 코일 전원을 끊는다. */
            step_release((uint8_t)(i + 1U));
        }
    }
}

void Step_StopAll(void)
{
    /* 전체 정지: 현재 위치를 새 목표 위치로 만들어 즉시 멈춘다. */
    target[0] = position[0];
    target[1] = position[1];
    step_release(1);
    step_release(2);
}

void Step_Stop(uint8_t motor)
{
    if (motor < 1U || motor > STEP_MOTOR_COUNT)
    {
        return;
    }

    target[motor - 1U] = position[motor - 1U];
    step_release(motor);
}

uint8_t Step_IsMoving(uint8_t motor)
{
    if (motor < 1U || motor > STEP_MOTOR_COUNT)
    {
        return 0U;
    }

    return (position[motor - 1U] != target[motor - 1U]) ? 1U : 0U;
}

int32_t Step_GetPosition(uint8_t motor)
{
    if (motor < 1U || motor > STEP_MOTOR_COUNT)
    {
        return 0;
    }

    return position[motor - 1U];
}
