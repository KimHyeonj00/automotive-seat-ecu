#include "pinchdetect.h"
#include "servo.h"
#include "step.h"

#define PINCH_SAMPLE_INTERVAL_MS        20U
#define PINCH_AVG_WINDOW_SIZE           10U

#define INA226_REG_SHUNT_V              0x01U
#define INA226_REG_BUS_V                0x02U
#define INA226_RSHUNT_OHM               0.1f

/* 1로 바꾸면 끼임 판단만 하고 실제 정지는 하지 않는 측정 전용 모드 */
#define PINCH_MEASURE_ONLY_MODE         1U

typedef struct
{
    uint8_t servo_ch;
    uint8_t step_motor;
    uint16_t i2c_addr;
    PinchDetect_Data_t *sensor;
} PinchSeatMap_t;

typedef struct
{
    float soft_current_A;       /* moving average 기준 */
    float hard_current_A;       /* 순간 전류 기준 */
    uint8_t soft_count_limit;
    uint8_t hard_count_limit;
    uint32_t start_ignore_ms;
    uint8_t soft_use_avg;
} PinchThreshold_t;

typedef struct
{
    uint8_t latched;
    uint8_t soft_count;
    uint8_t hard_count;
    uint8_t moving_prev;
    uint32_t motion_start_tick;
    uint32_t suspend_until_tick;

    float avg_buf[PINCH_AVG_WINDOW_SIZE];
    float avg_sum_A;
    uint8_t avg_index;
    uint8_t avg_count;
} PinchSeatState_t;

/* 전류 측정 디버그용 */
typedef struct
{
    uint8_t measuring;
    uint32_t sample_count;
    float current_sum_A;
    float current_avg_A;
    float current_max_A;
    float moving_avg_A;
    float bus_min_V;
} PinchMeasure_t;

PinchMeasure_t driver_pinch_measure;
PinchMeasure_t passenger_pinch_measure;

PinchDetect_Data_t driver_pinch_sensor;
PinchDetect_Data_t passenger_pinch_sensor;

static const PinchSeatMap_t seat_map[SEAT_COUNT] =
{
    [SEAT_DRIVER]    = { 1U, 1U, PINCHDETECT_ADDR_DRIVER,    &driver_pinch_sensor },
    [SEAT_PASSENGER] = { 2U, 2U, PINCHDETECT_ADDR_PASSENGER, &passenger_pinch_sensor }
};

/* soft_current_A는 최근 10개 샘플 이동평균 기준,
 * hard_current_A는 순간 전류 기준이다.
 */
static const PinchThreshold_t threshold_map[SEAT_COUNT] =
{
    [SEAT_DRIVER] =
    {
        .soft_current_A = 0.11f,
        .hard_current_A = 0.85f,
        .soft_count_limit = 3U,
        .hard_count_limit = 2U,
        .start_ignore_ms = 300U,
        .soft_use_avg = 1U

    },
    [SEAT_PASSENGER] =
    {
        .soft_current_A = 0.30f,
        .hard_current_A = 0.645f,
        .soft_count_limit = 8U,
        .hard_count_limit = 2U,
        .start_ignore_ms = 200U,
		.soft_use_avg = 0U
    }
};

static PinchSeatState_t seat_state[SEAT_COUNT];
static I2C_HandleTypeDef *ina_i2c;
static uint32_t last_sample_tick;

static uint8_t seat_is_valid(SeatId_t seat);
static PinchMeasure_t *seat_measure_ptr(SeatId_t seat);
static void measure_reset(SeatId_t seat);
static void measure_update(SeatId_t seat, const PinchDetect_Data_t *data);
static void moving_average_reset(PinchSeatState_t *state);
static float update_moving_average(PinchSeatState_t *state, float current_A);

static uint8_t seat_is_valid(SeatId_t seat)
{
    return ((uint8_t)seat < SEAT_COUNT) ? 1U : 0U;
}

static HAL_StatusTypeDef ina226_read_reg(uint16_t dev_addr, uint8_t reg_addr, uint16_t *value)
{
    uint8_t buf[2];
    HAL_StatusTypeDef ret;

    if (ina_i2c == 0 || value == 0)
    {
        return HAL_ERROR;
    }

    ret = HAL_I2C_Mem_Read(ina_i2c,
                           dev_addr,
                           reg_addr,
                           I2C_MEMADD_SIZE_8BIT,
                           buf,
                           2U,
                           20U);

    if (ret == HAL_OK)
    {
        *value = ((uint16_t)buf[0] << 8) | buf[1];
    }

    return ret;
}

static uint8_t ina226_read_current(uint16_t dev_addr, PinchDetect_Data_t *data)
{
    uint16_t raw_shunt;
    uint16_t raw_bus;
    int16_t signed_shunt;

    if (data == 0)
    {
        return 0U;
    }

    data->valid = 0U;

    if (ina226_read_reg(dev_addr, INA226_REG_SHUNT_V, &raw_shunt) != HAL_OK)
    {
        return 0U;
    }

    if (ina226_read_reg(dev_addr, INA226_REG_BUS_V, &raw_bus) != HAL_OK)
    {
        return 0U;
    }

    signed_shunt = (int16_t)raw_shunt;

    data->shunt_mV = (float)signed_shunt * 0.0025f;
    data->bus_V = (float)raw_bus * 0.00125f;
    data->current_A = (data->shunt_mV / 1000.0f) / INA226_RSHUNT_OHM;

    if (data->current_A < 0.0f)
    {
        data->current_A = -data->current_A;
    }

    data->valid = 1U;
    return 1U;
}

static void stop_seat(SeatId_t seat)
{
    Servo_EmergencyStop(seat_map[seat].servo_ch);

    /* 스텝모터 전류로 끼임 판단은 하지 않지만,
     * 같은 좌석에서 끼임이 발생하면 안전하게 회전 동작도 같이 멈춘다.
     */
    Step_Stop(seat_map[seat].step_motor);
}

static void moving_average_reset(PinchSeatState_t *state)
{
    uint8_t i;

    if (state == 0)
    {
        return;
    }

    state->avg_sum_A = 0.0f;
    state->avg_index = 0U;
    state->avg_count = 0U;

    for (i = 0U; i < PINCH_AVG_WINDOW_SIZE; i++)
    {
        state->avg_buf[i] = 0.0f;
    }
}

static float update_moving_average(PinchSeatState_t *state, float current_A)
{
    if (state == 0)
    {
        return 0.0f;
    }

    if (state->avg_count < PINCH_AVG_WINDOW_SIZE)
    {
        state->avg_buf[state->avg_index] = current_A;
        state->avg_sum_A += current_A;
        state->avg_count++;
    }
    else
    {
        state->avg_sum_A -= state->avg_buf[state->avg_index];
        state->avg_buf[state->avg_index] = current_A;
        state->avg_sum_A += current_A;
    }

    state->avg_index++;
    if (state->avg_index >= PINCH_AVG_WINDOW_SIZE)
    {
        state->avg_index = 0U;
    }

    return state->avg_sum_A / (float)state->avg_count;
}

static void clear_one(SeatId_t seat)
{
    seat_state[seat].latched = 0U;
    seat_state[seat].soft_count = 0U;
    seat_state[seat].hard_count = 0U;
    seat_state[seat].moving_prev = 0U;
    seat_state[seat].motion_start_tick = 0U;
    seat_state[seat].suspend_until_tick = 0U;
    moving_average_reset(&seat_state[seat]);
}

void PinchDetect_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t i;

    ina_i2c = hi2c;
    last_sample_tick = 0U;

    for (i = 0U; i < SEAT_COUNT; i++)
    {
        clear_one((SeatId_t)i);

        seat_map[i].sensor->valid = 0U;
        seat_map[i].sensor->shunt_mV = 0.0f;
        seat_map[i].sensor->bus_V = 0.0f;
        seat_map[i].sensor->current_A = 0.0f;
    }

    measure_reset(SEAT_DRIVER);
    measure_reset(SEAT_PASSENGER);
}

uint8_t PinchDetect_IsDetected(SeatId_t seat)
{
    if (seat_is_valid(seat) == 0U)
    {
        return 0U;
    }

    return seat_state[seat].latched;
}

void PinchDetect_Clear(SeatId_t seat)
{
    if (seat_is_valid(seat) == 0U)
    {
        return;
    }

    clear_one(seat);
}

void PinchDetect_Suspend(SeatId_t seat, uint32_t suspend_ms)
{
    uint32_t now;

    if (seat_is_valid(seat) == 0U)
    {
        return;
    }

    now = HAL_GetTick();

    seat_state[seat].latched = 0U;
    seat_state[seat].soft_count = 0U;
    seat_state[seat].hard_count = 0U;
    seat_state[seat].moving_prev = 0U;
    seat_state[seat].motion_start_tick = 0U;
    seat_state[seat].suspend_until_tick = now + suspend_ms;
    moving_average_reset(&seat_state[seat]);
}

static void process_one_seat(SeatId_t seat, uint32_t now)
{
    PinchSeatState_t *state = &seat_state[seat];
    const PinchSeatMap_t *map = &seat_map[seat];
    const PinchThreshold_t *threshold = &threshold_map[seat];
    uint8_t servo_moving;
    float current_A;
    float avg_A;
    float soft_check_A;

    if (state->latched != 0U)
    {
        return;
    }

    /* 복구 동작 중에는 일부러 반대 방향으로 움직이므로 끼임 판단을 잠시 중지한다. */
    if ((int32_t)(now - state->suspend_until_tick) < 0)
    {
        state->soft_count = 0U;
        state->hard_count = 0U;
        state->moving_prev = 0U;
        state->motion_start_tick = 0U;
        moving_average_reset(state);
        return;
    }

    servo_moving = Servo_IsMoving(map->servo_ch);

    /* 스텝모터는 INA226 회로에 연결하지 않았으므로,
     * 서보가 움직이지 않을 때는 전류 측정/끼임 판단을 하지 않는다.
     */
    if (servo_moving == 0U)
    {
        if (state->moving_prev != 0U)
        {
            /* 서보가 방금 이동을 끝낸 순간.
             * 필요하면 여기 __NOP()에 브레이크포인트를 걸어 측정값을 확인한다.
             */
            seat_measure_ptr(seat)->measuring = 0U;
            __NOP();
        }

        state->soft_count = 0U;
        state->hard_count = 0U;
        state->moving_prev = 0U;
        state->motion_start_tick = 0U;
        moving_average_reset(state);
        return;
    }

    /* 서보가 막 움직이기 시작한 순간 기록 */
    if (state->moving_prev == 0U)
    {
        state->moving_prev = 1U;
        state->motion_start_tick = now;
        state->soft_count = 0U;
        state->hard_count = 0U;
        moving_average_reset(state);
        return;
    }

    /* 시작 직후 기동 전류 무시 */
    if ((now - state->motion_start_tick) < threshold->start_ignore_ms)
    {
        state->soft_count = 0U;
        state->hard_count = 0U;
        moving_average_reset(state);
        return;
    }

    if (ina226_read_current(map->i2c_addr, map->sensor) == 0U)
    {
        state->soft_count = 0U;
        state->hard_count = 0U;
        moving_average_reset(state);
        return;
    }

    measure_update(seat, map->sensor);

    current_A = map->sensor->current_A;
    avg_A = update_moving_average(state, current_A);
    seat_measure_ptr(seat)->moving_avg_A = avg_A;

    if (threshold->soft_use_avg != 0U)
    {
        soft_check_A = avg_A;
    }
    else
    {
        soft_check_A = current_A;
    }

    /* hard 조건은 순간 전류 기준이다. 정상 피크와 구분되도록 높게 잡는다. */
    if (current_A >= threshold->hard_current_A)
    {
        if (state->hard_count < 255U)
        {
            state->hard_count++;
        }
    }
    else
    {
        state->hard_count = 0U;
    }

    /* soft 조건은 최근 10개 샘플 이동평균 기준이다. */
    if (soft_check_A >= threshold->soft_current_A)
    {
        if (state->soft_count < 255U)
        {
            state->soft_count++;
        }
    }
    else
    {
        state->soft_count = 0U;
    }

    if (state->hard_count >= threshold->hard_count_limit ||
        state->soft_count >= threshold->soft_count_limit)
    {
#if PINCH_MEASURE_ONLY_MODE
        /* 측정 모드에서는 정지하지 않음 */
#else
        state->latched = 1U;
        stop_seat(seat);
#endif
    }
}

void PinchDetect_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t i;

    if ((now - last_sample_tick) < PINCH_SAMPLE_INTERVAL_MS)
    {
        return;
    }

    last_sample_tick = now;

    for (i = 0U; i < SEAT_COUNT; i++)
    {
        process_one_seat((SeatId_t)i, now);
    }
}

static PinchMeasure_t *seat_measure_ptr(SeatId_t seat)
{
    return (seat == SEAT_DRIVER) ? &driver_pinch_measure : &passenger_pinch_measure;
}

static void measure_reset(SeatId_t seat)
{
    PinchMeasure_t *m = seat_measure_ptr(seat);

    m->measuring = 0U;
    m->sample_count = 0U;
    m->current_sum_A = 0.0f;
    m->current_avg_A = 0.0f;
    m->current_max_A = 0.0f;
    m->moving_avg_A = 0.0f;
    m->bus_min_V = 99.0f;
}

static void measure_update(SeatId_t seat, const PinchDetect_Data_t *data)
{
    PinchMeasure_t *m = seat_measure_ptr(seat);

    if (data == 0 || data->valid == 0U)
    {
        return;
    }

    if (m->measuring == 0U)
    {
        m->measuring = 1U;
        m->sample_count = 0U;
        m->current_sum_A = 0.0f;
        m->current_avg_A = 0.0f;
        m->current_max_A = 0.0f;
        m->moving_avg_A = 0.0f;
        m->bus_min_V = 99.0f;
    }

    m->sample_count++;
    m->current_sum_A += data->current_A;
    m->current_avg_A = m->current_sum_A / (float)m->sample_count;

    if (data->current_A > m->current_max_A)
    {
        m->current_max_A = data->current_A;
    }

    if (data->bus_V < m->bus_min_V)
    {
        m->bus_min_V = data->bus_V;
    }
}
