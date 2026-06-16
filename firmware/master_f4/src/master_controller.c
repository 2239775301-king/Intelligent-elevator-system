#include "../inc/master_controller.h"

/* +2 cost makes moving-away cars less preferred without starving distant idle cars. */
#define DIRECTION_MISMATCH_PENALTY 2U

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t previous_ms) {
    return (uint32_t)(now_ms - previous_ms);
}

static uint8_t clamp_floor(uint8_t floor) {
    return (floor > ELEVATOR_MAX_FLOOR) ? ELEVATOR_MAX_FLOOR : floor;
}

static void set_hall_call(master_controller_t* controller, hall_call_t call) {
    uint32_t mask = (1UL << clamp_floor(call.floor));
    if (call.direction == DIRECTION_DOWN) {
        controller->pending_down_mask |= mask;
        return;
    }
    controller->pending_up_mask |= mask;
}

static bool get_next_call(const master_controller_t* controller, hall_call_t* out_call) {
    for (uint8_t f = 0U; f <= ELEVATOR_MAX_FLOOR; ++f) {
        uint32_t mask = (1UL << f);
        if ((controller->pending_up_mask & mask) != 0U) {
            out_call->floor = f;
            out_call->direction = DIRECTION_UP;
            return true;
        }
        if ((controller->pending_down_mask & mask) != 0U) {
            out_call->floor = f;
            out_call->direction = DIRECTION_DOWN;
            return true;
        }
    }
    return false;
}

static void clear_call(master_controller_t* controller, hall_call_t call) {
    uint32_t mask = ~(1UL << clamp_floor(call.floor));
    if (call.direction == DIRECTION_DOWN) {
        controller->pending_down_mask &= mask;
        return;
    }
    controller->pending_up_mask &= mask;
}

static uint8_t select_best_node(const master_controller_t* controller, hall_call_t call) {
    uint8_t best_node = 0U;
    uint8_t best_cost = 0xFFU;

    for (uint8_t node = 1U; node <= ELEVATOR_MAX_NODES; ++node) {
        const master_node_state_t* n = &controller->nodes[node];
        if (!n->online || n->status.mode == ELEVATOR_MODE_FAULT) {
            continue;
        }

        uint8_t distance = (n->status.floor > call.floor)
            ? (uint8_t)(n->status.floor - call.floor)
            : (uint8_t)(call.floor - n->status.floor);
        /* Prefer cars already moving toward the request by adding a small cost to cars moving away. */
        uint8_t penalty = 0U;
        bool moving_away = (n->status.motion == DIRECTION_UP && n->status.floor > call.floor)
            || (n->status.motion == DIRECTION_DOWN && n->status.floor < call.floor);
        if (n->status.mode == ELEVATOR_MODE_MOVING && moving_away) {
            penalty = DIRECTION_MISMATCH_PENALTY;
        }

        uint8_t cost = (uint8_t)(distance + penalty);
        if (cost < best_cost) {
            best_cost = cost;
            best_node = node;
        }
    }

    return best_node;
}

static can_tx_frame_t build_assign_frame(uint8_t node, hall_call_t call) {
    can_tx_frame_t frame = {0};
    frame.id = can_id_command(node);
    frame.dlc = 8U;
    frame.data[0] = clamp_floor(call.floor);
    frame.data[1] = (uint8_t)call.direction;
    frame.data[2] = COMMAND_CODE_ASSIGN_TARGET;
    return frame;
}

void Master_Init(master_controller_t* controller, uint32_t heartbeat_timeout_ms) {
    if (controller == NULL) {
        return;
    }

    for (uint8_t i = 0U; i <= ELEVATOR_MAX_NODES; ++i) {
        controller->nodes[i].online = false;
        controller->nodes[i].last_heartbeat_ms = 0U;
        controller->nodes[i].status.node_id = i;
        controller->nodes[i].status.floor = 0U;
        controller->nodes[i].status.motion = DIRECTION_IDLE;
        controller->nodes[i].status.mode = ELEVATOR_MODE_IDLE;
        controller->nodes[i].status.door = DOOR_CLOSED;
        controller->nodes[i].status.fault_code = 0U;
    }

    controller->pending_up_mask = 0U;
    controller->pending_down_mask = 0U;
    controller->heartbeat_timeout_ms = heartbeat_timeout_ms;
}

void Master_OnHeartbeat(master_controller_t* controller, uint8_t node_id, uint32_t now_ms) {
    if (controller == NULL || !is_valid_node(node_id)) {
        return;
    }

    controller->nodes[node_id].online = true;
    controller->nodes[node_id].last_heartbeat_ms = now_ms;
}

void Master_OnStatus(master_controller_t* controller, const elevator_status_t* status, uint32_t now_ms) {
    if (controller == NULL || status == NULL || !is_valid_node(status->node_id)) {
        return;
    }

    master_node_state_t* node = &controller->nodes[status->node_id];
    node->status = *status;
    node->online = true;
    node->last_heartbeat_ms = now_ms;
}

void Master_OnHallCall(master_controller_t* controller, const hall_call_t* call) {
    if (controller == NULL || call == NULL) {
        return;
    }
    set_hall_call(controller, *call);
}

size_t Master_Tick(
    master_controller_t* controller,
    uint32_t now_ms,
    can_tx_frame_t* out_frames,
    size_t out_capacity) {
    if (controller == NULL || out_frames == NULL || out_capacity == 0U) {
        return 0U;
    }

    for (uint8_t node = 1U; node <= ELEVATOR_MAX_NODES; ++node) {
        master_node_state_t* n = &controller->nodes[node];
        if (n->online && elapsed_ms(now_ms, n->last_heartbeat_ms) > controller->heartbeat_timeout_ms) {
            n->online = false;
            n->status.mode = ELEVATOR_MODE_FAULT;
            n->status.fault_code = FAULT_CODE_HEARTBEAT_TIMEOUT;
        }
    }

    hall_call_t call = {0};
    if (!get_next_call(controller, &call)) {
        return 0U;
    }

    uint8_t node = select_best_node(controller, call);
    if (node == 0U) {
        return 0U;
    }

    out_frames[0] = build_assign_frame(node, call);
    clear_call(controller, call);
    return 1U;
}
