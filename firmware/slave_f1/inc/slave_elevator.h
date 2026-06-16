#ifndef SLAVE_ELEVATOR_H
#define SLAVE_ELEVATOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../../common/can_protocol.h"

typedef struct {
    uint8_t node_id;
    uint8_t current_floor;
    uint8_t target_floor;
    direction_t motion;
    elevator_mode_t mode;
    door_state_t door;
    uint8_t fault_code;
    uint32_t last_motion_ms;
    uint32_t door_open_ms;
} slave_elevator_t;

void Slave_Init(slave_elevator_t* elevator, uint8_t node_id, uint8_t initial_floor);
/* direction is reserved for future dispatch strategy constraints. */
void Slave_ProcessAssignCommand(
    slave_elevator_t* elevator,
    uint8_t target_floor,
    direction_t direction,
    uint32_t now_ms);
void Slave_SetFault(slave_elevator_t* elevator, uint8_t fault_code);
void Slave_ClearFault(slave_elevator_t* elevator);
void Slave_Tick(slave_elevator_t* elevator, uint32_t now_ms);

void Slave_BuildStatusFrame(const slave_elevator_t* elevator, can_tx_frame_t* out_frame);
void Slave_BuildHeartbeatFrame(const slave_elevator_t* elevator, can_tx_frame_t* out_frame);

#endif
