#ifndef MASTER_CONTROLLER_H
#define MASTER_CONTROLLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../../common/can_protocol.h"

typedef struct {
    bool online;
    uint32_t last_heartbeat_ms;
    elevator_status_t status;
} master_node_state_t;

typedef struct {
    master_node_state_t nodes[ELEVATOR_MAX_NODES + 1U];
    uint32_t pending_up_mask;
    uint32_t pending_down_mask;
    uint32_t heartbeat_timeout_ms;
    uint32_t total_dispatch_count;
} master_controller_t;

typedef struct {
    uint32_t pending_call_count;
    uint32_t online_node_count;
    uint32_t timeout_isolation_count;
    uint32_t total_dispatch_count;
    bool has_schedulable_node;
} master_observer_t;

void Master_Init(master_controller_t* controller, uint32_t heartbeat_timeout_ms);
void Master_OnHeartbeat(master_controller_t* controller, uint8_t node_id, uint32_t now_ms);
void Master_OnStatus(master_controller_t* controller, const elevator_status_t* status, uint32_t now_ms);
void Master_OnHallCall(master_controller_t* controller, const hall_call_t* call);

size_t Master_Tick(
    master_controller_t* controller,
    uint32_t now_ms,
    can_tx_frame_t* out_frames,
    size_t out_capacity);

void Master_GetObserver(const master_controller_t* controller, master_observer_t* out_observer);

#endif
