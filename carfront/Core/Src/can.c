#include "can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan;

#define CAN_RX_CMD_QUEUE_SIZE  8U

typedef struct
{
    SeatCommand_t buf[CAN_RX_CMD_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t overflow;
} CanCommandQueue_t;

static volatile CanCommandQueue_t seat_cmd_q;
static volatile GearStatus_t gear_rx;
static volatile uint8_t gear_pending;
static volatile SafeAbort_t abort_rx;
static volatile uint8_t abort_pending;

static uint32_t tx_mailbox;

static uint16_t can_std_filter_value(uint16_t std_id)
{
    return (uint16_t)(std_id << 5);
}

static uint8_t driver_cmd_checksum(const uint8_t data[4])
{
    return (uint8_t)(data[0] + data[1] + data[2]);
}

static void queue_clear(void)
{
    seat_cmd_q.head = 0U;
    seat_cmd_q.tail = 0U;
    seat_cmd_q.overflow = 0U;
}

static void queue_push_from_isr(const SeatCommand_t *cmd)
{
    uint8_t next_head;

    if (cmd == 0)
    {
        return;
    }

    next_head = (uint8_t)((seat_cmd_q.head + 1U) % CAN_RX_CMD_QUEUE_SIZE);

    /* Queue full: 오래된 명령을 버리고 최신 명령을 남긴다. */
    if (next_head == seat_cmd_q.tail)
    {
        seat_cmd_q.tail = (uint8_t)((seat_cmd_q.tail + 1U) % CAN_RX_CMD_QUEUE_SIZE);
        seat_cmd_q.overflow = 1U;
    }

    seat_cmd_q.buf[seat_cmd_q.head] = *cmd;
    seat_cmd_q.head = next_head;
}

static void can_filter_init(void)
{
    CAN_FilterTypeDef f = {0};

    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDLIST;
    f.FilterScale = CAN_FILTERSCALE_16BIT;
    f.FilterIdHigh = can_std_filter_value(CAN_ID_SAFE_ABORT);
    f.FilterIdLow = can_std_filter_value(CAN_ID_GEAR_STATUS);
    f.FilterMaskIdHigh = can_std_filter_value(CAN_ID_DRIVER_SEAT_CMD);
    f.FilterMaskIdLow = can_std_filter_value(CAN_ID_PASSENGER_SEAT_CMD);
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_AppStart(void)
{
    queue_clear();
    gear_pending = 0U;
    abort_pending = 0U;

    can_filter_init();

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t CAN_AppPopSeatCommand(SeatCommand_t *cmd)
{
    if (cmd == 0)
    {
        return 0U;
    }

    __disable_irq();

    if (seat_cmd_q.head == seat_cmd_q.tail)
    {
        __enable_irq();
        return 0U;
    }

    *cmd = seat_cmd_q.buf[seat_cmd_q.tail];
    seat_cmd_q.tail = (uint8_t)((seat_cmd_q.tail + 1U) % CAN_RX_CMD_QUEUE_SIZE);

    __enable_irq();
    return 1U;
}

uint8_t CAN_AppPopGearStatus(GearStatus_t *status)
{
    if (status == 0)
    {
        return 0U;
    }

    __disable_irq();

    if (gear_pending == 0U)
    {
        __enable_irq();
        return 0U;
    }

    *status = gear_rx;
    gear_pending = 0U;

    __enable_irq();
    return 1U;
}

uint8_t CAN_AppPopSafeAbort(SafeAbort_t *abort_msg)
{
    if (abort_msg == 0)
    {
        return 0U;
    }

    __disable_irq();

    if (abort_pending == 0U)
    {
        __enable_irq();
        return 0U;
    }

    *abort_msg = abort_rx;
    abort_pending = 0U;

    __enable_irq();
    return 1U;
}

void CAN_AppSendSeatStatus(SeatId_t seat,
                           uint8_t curr_recline,
                           uint8_t curr_rotation,
                           uint8_t pinch_detected)
{
    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[3];

    header.StdId = (seat == SEAT_DRIVER) ? CAN_ID_DRIVER_SEAT_STATUS : CAN_ID_PASSENGER_SEAT_STATUS;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 3U;
    header.TransmitGlobalTime = DISABLE;

    data[0] = curr_recline;
    data[1] = curr_rotation;
    data[2] = (uint8_t)(pinch_detected & 0x01U);

    (void)HAL_CAN_AddTxMessage(&hcan, &header, data, &tx_mailbox);
}

static void decode_driver_cmd(const uint8_t data[4], SeatCommand_t *cmd)
{
    cmd->seat = SEAT_DRIVER;
    cmd->recline_angle = data[0];
    cmd->rotation_angle = data[1];
    cmd->rolling_counter = (uint8_t)(data[2] & 0x0FU);
    cmd->checksum_ok = (data[3] == driver_cmd_checksum(data)) ? 1U : 0U;
}

static void decode_passenger_cmd(const uint8_t data[2], SeatCommand_t *cmd)
{
    cmd->seat = SEAT_PASSENGER;
    cmd->recline_angle = data[0];
    cmd->rotation_angle = data[1];
    cmd->rolling_counter = 0U;
    cmd->checksum_ok = 1U;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can_handle)
{
    CAN_RxHeaderTypeDef header;
    SeatCommand_t cmd;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(can_handle, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
        return;
    }

    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA)
    {
        return;
    }

    switch (header.StdId)
    {
    case CAN_ID_SAFE_ABORT:
        if (header.DLC >= 3U)
        {
            abort_rx.stop_flag = (uint8_t)(data[0] & 0x01U);
            abort_rx.source_id = data[1];
            abort_rx.reason_code = data[2];
            abort_pending = 1U;
        }
        break;

    case CAN_ID_GEAR_STATUS:
        if (header.DLC >= 1U)
        {
            gear_rx.gear = (uint8_t)(data[0] & 0x03U);
            gear_pending = 1U;
        }
        break;

    case CAN_ID_DRIVER_SEAT_CMD:
        if (header.DLC >= 4U)
        {
            decode_driver_cmd(data, &cmd);
            queue_push_from_isr(&cmd);
        }
        break;

    case CAN_ID_PASSENGER_SEAT_CMD:
        if (header.DLC >= 2U)
        {
            decode_passenger_cmd(data, &cmd);
            queue_push_from_isr(&cmd);
        }
        break;

    default:
        break;
    }
}
