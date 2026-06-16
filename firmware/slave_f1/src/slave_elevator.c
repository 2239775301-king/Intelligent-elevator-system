#include "../inc/slave_elevator.h"

/* Simulated travel time for moving one floor. */
#define FLOOR_TRAVEL_MS 1000U
/* Simulated time that doors remain open after arrival. */
#define DOOR_HOLD_MS    2000U

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t previous_ms) {
    return now_ms - previous_ms;
}

static uint8_t clamp_floor(uint8_t floor) {
    return (floor > ELEVATOR_MAX_FLOOR) ? ELEVATOR_MAX_FLOOR : floor;
}

static void update_motion_direction(slave_elevator_t* elevator) {
    if (elevator->current_floor < elevator->target_floor) {
        elevator->motion = DIRECTION_UP;
    } else if (elevator->current_floor > elevator->target_floor) {
        elevator->motion = DIRECTION_DOWN;
    } else {
        elevator->motion = DIRECTION_IDLE;
    }
}

void Slave_Init(slave_elevator_t* elevator, uint8_t node_id, uint8_t initial_floor) {
    if (elevator == NULL) {
        return;
    }

    elevator->node_id = node_id;
    elevator->current_floor = clamp_floor(initial_floor);
    elevator->target_floor = elevator->current_floor;
    elevator->motion = DIRECTION_IDLE;
    elevator->mode = ELEVATOR_MODE_IDLE;
    elevator->door = DOOR_CLOSED;
    elevator->fault_code = 0U;
    elevator->last_motion_ms = 0U;
    elevator->door_open_ms = 0U;
}

void Slave_ProcessAssignCommand(slave_elevator_t* elevator, uint8_t target_floor, direction_t direction) {
    (void)direction;

    if (elevator == NULL || elevator->mode == ELEVATOR_MODE_FAULT) {
        return;
    }

    elevator->target_floor = clamp_floor(target_floor);
    update_motion_direction(elevator);

    if (elevator->motion == DIRECTION_IDLE) {
        elevator->door = DOOR_OPENED;
        elevator->mode = ELEVATOR_MODE_DOOR_OPEN;
        return;
    }

    elevator->door = DOOR_CLOSED;
    elevator->mode = ELEVATOR_MODE_MOVING;
}

void Slave_SetFault(slave_elevator_t* elevator, uint8_t fault_code) {
    if (elevator == NULL) {
        return;
    }

    elevator->mode = ELEVATOR_MODE_FAULT;
    elevator->fault_code = fault_code;
    elevator->motion = DIRECTION_IDLE;
    elevator->door = DOOR_CLOSED;
}

void Slave_ClearFault(slave_elevator_t* elevator) {
    if (elevator == NULL || elevator->mode != ELEVATOR_MODE_FAULT) {
        return;
    }

    elevator->fault_code = 0U;
    elevator->mode = ELEVATOR_MODE_IDLE;
}

void Slave_Tick(slave_elevator_t* elevator, uint32_t now_ms) {
    if (elevator == NULL || elevator->mode == ELEVATOR_MODE_FAULT) {
        return;
    }

    if (elevator->mode == ELEVATOR_MODE_MOVING) {
        if (elapsed_ms(now_ms, elevator->last_motion_ms) < FLOOR_TRAVEL_MS) {
            return;
        }
        elevator->last_motion_ms = now_ms;

        if (elevator->motion == DIRECTION_UP && elevator->current_floor < ELEVATOR_MAX_FLOOR) {
            elevator->current_floor++;
        } else if (elevator->motion == DIRECTION_DOWN && elevator->current_floor > 0U) {
            elevator->current_floor--;
        }

        update_motion_direction(elevator);
        if (elevator->motion == DIRECTION_IDLE) {
            elevator->mode = ELEVATOR_MODE_DOOR_OPEN;
            elevator->door = DOOR_OPENED;
            elevator->door_open_ms = now_ms;
        }
        return;
    }

    if (elevator->mode == ELEVATOR_MODE_DOOR_OPEN) {
        if (elapsed_ms(now_ms, elevator->door_open_ms) >= DOOR_HOLD_MS) {
            elevator->door = DOOR_CLOSED;
            elevator->mode = ELEVATOR_MODE_IDLE;
        }
    }
}

void Slave_BuildStatusFrame(const slave_elevator_t* elevator, can_tx_frame_t* out_frame) {
    if (elevator == NULL || out_frame == NULL) {
        return;
    }

    out_frame->id = can_id_status(elevator->node_id);
    out_frame->dlc = 8U;
    out_frame->data[0] = elevator->node_id;
    out_frame->data[1] = elevator->current_floor;
    out_frame->data[2] = (uint8_t)elevator->motion;
    out_frame->data[3] = (uint8_t)elevator->mode;
    out_frame->data[4] = (uint8_t)elevator->door;
    out_frame->data[5] = elevator->fault_code;
    out_frame->data[6] = 0U;
    out_frame->data[7] = 0U;
}

void Slave_BuildHeartbeatFrame(const slave_elevator_t* elevator, can_tx_frame_t* out_frame) {
    if (elevator == NULL || out_frame == NULL) {
        return;
    }

    out_frame->id = can_id_heartbeat(elevator->node_id);
    out_frame->dlc = 8U;
    out_frame->data[0] = elevator->node_id;
    out_frame->data[1] = (uint8_t)elevator->mode;
    out_frame->data[2] = (uint8_t)elevator->door;
    out_frame->data[3] = elevator->fault_code;
    out_frame->data[4] = 0U;
    out_frame->data[5] = 0U;
    out_frame->data[6] = 0U;
    out_frame->data[7] = 0U;
}
